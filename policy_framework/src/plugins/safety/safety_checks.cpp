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

#include "pluginlib/class_list_macros.hpp"
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
              ctx.name + ": interface '" + interface + "' needs " + interface + ".min and/or " +
              interface + ".max");
    }
    if (bounds.min + bounds.hysteresis > bounds.max - bounds.hysteresis) {
      throw std::runtime_error(
              ctx.name + ": " + interface + ".hysteresis leaves no value to pass again within [" +
              interface + ".min, " + interface + ".max]");
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
  reason = state.names()[first->index] + " is " + std::to_string(first_value) + ", outside [" +
    std::to_string(first->min + first_margin) + ", " + std::to_string(first->max - first_margin) +
    "]";
  if (failing > 1) {reason += " (" + std::to_string(failing) + " entries failing)";}
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
              ctx.name + ": interface '" + interface + "' needs " + interface +
              ".max_delta above zero");
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
      reason = state.names()[e.index] + " jumped from " + std::to_string(e.previous) + " to " +
        std::to_string(v) + ", more than " + std::to_string(e.max_delta);
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
  reason = "mock fault";
  if (!msg->reason.empty()) {reason += ": " + msg->reason;}
  if (msg->reaction.empty()) {return {false, std::nullopt};}
  const auto reaction = fault_reaction_from_string(msg->reaction);
  if (!reaction) {
    reason += " (unknown reaction '" + msg->reaction + "': error)";
    return {false, FaultReaction::ERROR};
  }
  return {false, reaction};
}

void MockSafetyCheck::reset() {forgotten_ = topic_.get().msg;}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::StateBoundsCheck, policy_framework::SafetyCheck)
PLUGINLIB_EXPORT_CLASS(policy_framework::JumpCheck, policy_framework::SafetyCheck)
PLUGINLIB_EXPORT_CLASS(policy_framework::MockSafetyCheck, policy_framework::SafetyCheck)
