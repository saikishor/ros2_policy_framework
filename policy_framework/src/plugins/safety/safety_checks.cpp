// Copyright 2026 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "policy_framework/plugins/safety/safety_checks.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <fmt/format.h>

#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/encoder_mismatch_check_parameters.hpp"
#include "policy_framework/jump_check_parameters.hpp"
#include "policy_framework/mock_safety_check_parameters.hpp"
#include "policy_framework/state_bounds_check_parameters.hpp"

namespace policy_framework
{
namespace
{
// The parameter defaults: an unset bound.
constexpr double kUnbounded = 1.0e300;
}  // namespace

void StateBoundsCheck::configure(const PluginContext & ctx, const DataStore &, DataStore & state)
{
  const auto params = state_bounds_check::ParamListener(ctx.node, ctx.name).get_params();
  entries_.clear();
  trigger_cycles_ = static_cast<int>(params.trigger_cycles);
  escalation_count_ = static_cast<int>(params.escalation.count);
  // The parameter validation allows only the names it knows.
  escalation_reaction_ = *fault_reaction_from_string(params.escalation.on_fault);
  for (const auto & interface : params.interfaces) {
    const auto & bounds = params.interfaces_map.at(interface);
    if (bounds.min <= -kUnbounded && bounds.max >= kUnbounded) {
      throw std::runtime_error(
              fmt::format(
                "{0}: interface '{1}' needs {1}.min and/or {1}.max", ctx.name, interface));
    }
    if (bounds.min + bounds.hysteresis > bounds.max - bounds.hysteresis) {
      throw std::runtime_error(
              fmt::format(
                "{0}: {1}.hysteresis leaves no value to pass again within [{1}.min, {1}.max]",
                ctx.name, interface));
    }
    for (const auto & prefix : params.prefixes) {
      entries_.push_back(
        {state.add(prefix + "/" + interface), bounds.min, bounds.max, bounds.hysteresis});
    }
  }
}

SafetyCheck::Verdict StateBoundsCheck::check(
  const DataStore & state, double, std::string & reason)
{
  // Every entry is evaluated, also after a failing one, to count the failing entries and keep
  // the counters of all of them current.
  int failing = 0;
  const Entry * first = nullptr;
  double first_value = 0.0;
  double first_margin = 0.0;
  for (auto & e : entries_) {
    if (!state.received(e.index)) {continue;}
    const double v = state.get(e.index);
    const double margin = e.failed ? e.hysteresis : 0.0;
    // Written so that NaN fails: every comparison with it is false.
    if (v >= e.min + margin && v <= e.max - margin) {
      e.violations = 0;
      e.failed = false;
    } else if (e.failed || ++e.violations >= trigger_cycles_) {
      e.failed = true;
      if (!first) {
        first = &e;
        first_value = v;
        first_margin = margin;
      }
      ++failing;
    }
  }
  if (!first) {return {};}
  // The bounds it has to be back within: narrowed by the hysteresis once it failed.
  reason = fmt::format(
    "{} is {}, outside [{}, {}]", state.names()[first->index], first_value,
    first->min + first_margin, first->max - first_margin);
  if (failing > 1) {reason += fmt::format(" ({} entries failing)", failing);}
  if (escalation_count_ > 0 && failing >= escalation_count_) {
    return {false, escalation_reaction_};
  }
  return {false, std::nullopt};
}

std::vector<FaultReaction> StateBoundsCheck::possible_reactions() const
{
  if (escalation_count_ > 0) {return {escalation_reaction_};}
  return {};
}

void StateBoundsCheck::reset()
{
  for (auto & e : entries_) {
    e.violations = 0;
    e.failed = false;
  }
}

void JumpCheck::configure(const PluginContext & ctx, const DataStore &, DataStore & state)
{
  const auto params = jump_check::ParamListener(ctx.node, ctx.name).get_params();
  entries_.clear();
  for (const auto & interface : params.interfaces) {
    const double max_delta = params.interfaces_map.at(interface).max_delta;
    if (max_delta <= 0.0) {
      throw std::runtime_error(
              fmt::format(
                "{0}: interface '{1}' needs {1}.max_delta above zero", ctx.name, interface));
    }
    for (const auto & prefix : params.prefixes) {
      entries_.push_back({state.add(prefix + "/" + interface), max_delta});
    }
  }
}

SafetyCheck::Verdict JumpCheck::check(const DataStore & state, double, std::string & reason)
{
  for (auto & e : entries_) {
    if (!state.received(e.index)) {continue;}
    const double v = state.get(e.index);
    // isfinite() for a first value; the negated comparison also fails a NaN difference.
    if (!std::isfinite(v) || (e.has_previous && !(std::abs(v - e.previous) <= e.max_delta))) {
      reason = fmt::format(
        "{} jumped from {} to {}, more than {}", state.names()[e.index], e.previous, v,
        e.max_delta);
      return {false, std::nullopt};
    }
    e.previous = v;
    e.has_previous = true;
  }
  return {};
}

void JumpCheck::reset()
{
  for (auto & e : entries_) {e.has_previous = false;}
}

void EncoderMismatchCheck::configure(
  const PluginContext & ctx, const DataStore &, DataStore & state)
{
  const auto params = encoder_mismatch_check::ParamListener(ctx.node, ctx.name).get_params();
  tolerance_ = params.tolerance;
  persist_ = params.persist;
  persisted_reason_.clear();
  joints_.clear();
  for (const auto & joint : params.joints) {
    joints_.push_back({state.add(joint + "/absolute_position"), state.add(joint + "/position")});
  }
}

SafetyCheck::Verdict EncoderMismatchCheck::check(
  const DataStore & state, double, std::string & reason)
{
  if (!persisted_reason_.empty()) {
    reason = persisted_reason_;
    return {false, std::nullopt};
  }
  for (const auto & j : joints_) {
    if (!state.received(j.absolute) || !state.received(j.position)) {continue;}
    const double absolute = state.get(j.absolute);
    const double position = state.get(j.position);
    // Written so that a NaN on either side fails: every comparison with it is false.
    if (std::abs(absolute - position) <= tolerance_) {continue;}
    reason = fmt::format(
      "{} is {} but {} is {}, more than {} apart", state.names()[j.absolute], absolute,
      state.names()[j.position], position, tolerance_);
    if (persist_) {
      persisted_reason_ = fmt::format("{} (persists until reactivation or ~/reset_safety)", reason);
    }
    return {false, std::nullopt};
  }
  return {};
}

void EncoderMismatchCheck::reset() {persisted_reason_.clear();}

void MockSafetyCheck::configure(const PluginContext & ctx, const DataStore &, DataStore &)
{
  const auto params = mock_safety_check::ParamListener(ctx.node, ctx.name).get_params();
  reactions_.clear();
  // The parameter validation allows only the names it knows.
  for (const auto & r : params.reactions) {reactions_.push_back(*fault_reaction_from_string(r));}
  topic_.subscribe(ctx.node, topic_parameter(ctx, "~/" + ctx.name));
}

SafetyCheck::Verdict MockSafetyCheck::check(const DataStore &, double now, std::string & reason)
{
  const auto latest = topic_.get();
  const auto & msg = latest.msg;
  if (!msg || !msg->fault || msg == forgotten_) {return {};}
  if (msg->duration > 0.0 && now - latest.receive_time > msg->duration) {return {};}
  reason = msg->reason.empty() ? "mock fault" : fmt::format("mock fault: {}", msg->reason);
  if (msg->reaction.empty()) {return {false, std::nullopt};}
  const auto reaction = fault_reaction_from_string(msg->reaction);
  if (!reaction) {
    reason += fmt::format(" (unknown reaction '{}': error)", msg->reaction);
    return {false, FaultReaction::ERROR};
  }
  return {false, reaction};
}

void MockSafetyCheck::reset() {forgotten_ = topic_.get().msg;}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::StateBoundsCheck, policy_framework::SafetyCheck)
PLUGINLIB_EXPORT_CLASS(policy_framework::JumpCheck, policy_framework::SafetyCheck)
PLUGINLIB_EXPORT_CLASS(policy_framework::EncoderMismatchCheck, policy_framework::SafetyCheck)
PLUGINLIB_EXPORT_CLASS(policy_framework::MockSafetyCheck, policy_framework::SafetyCheck)
