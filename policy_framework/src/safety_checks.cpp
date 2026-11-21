// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/safety_checks.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/jump_check_parameters.hpp"
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
  for (const auto & interface : params.interfaces) {
    const auto & bounds = params.interfaces_map.at(interface);
    if (bounds.min <= -kUnbounded && bounds.max >= kUnbounded) {
      throw std::runtime_error(
              ctx.name + ": interface '" + interface + "' needs " + interface + ".min and/or " +
              interface + ".max");
    }
    for (const auto & prefix : params.prefixes) {
      entries_.push_back({state.add(prefix + "/" + interface), bounds.min, bounds.max});
    }
  }
}

bool StateBoundsCheck::check(const DataStore & state, double, std::string & reason)
{
  for (auto & e : entries_) {
    if (!state.received(e.index)) {continue;}
    const double v = state.get(e.index);
    // Written so that NaN fails: every comparison with it is false.
    if (v >= e.min && v <= e.max) {
      e.violations = 0;
    } else if (++e.violations >= trigger_cycles_) {
      reason = state.names()[e.index] + " is " + std::to_string(v) + ", outside [" +
        std::to_string(e.min) + ", " + std::to_string(e.max) + "]";
      return false;
    }
  }
  return true;
}

void StateBoundsCheck::reset()
{
  for (auto & e : entries_) {e.violations = 0;}
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

bool JumpCheck::check(const DataStore & state, double, std::string & reason)
{
  for (auto & e : entries_) {
    if (!state.received(e.index)) {continue;}
    const double v = state.get(e.index);
    // isfinite() for a first value; the negated comparison also fails a NaN difference.
    if (!std::isfinite(v) || (e.has_previous && !(std::abs(v - e.previous) <= e.max_delta))) {
      reason = state.names()[e.index] + " jumped from " + std::to_string(e.previous) + " to " +
        std::to_string(v) + ", more than " + std::to_string(e.max_delta);
      return false;
    }
    e.previous = v;
    e.has_previous = true;
  }
  return true;
}

void JumpCheck::reset()
{
  for (auto & e : entries_) {e.has_previous = false;}
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::StateBoundsCheck, policy_framework::SafetyCheck)
PLUGINLIB_EXPORT_CLASS(policy_framework::JumpCheck, policy_framework::SafetyCheck)
