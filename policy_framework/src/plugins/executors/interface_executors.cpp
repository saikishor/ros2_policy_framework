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

#include "policy_framework/plugins/executors/interface_executors.hpp"

#include "core/entry_name.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/pd_effort_executor_parameters.hpp"
#include "policy_framework/reference_interface_executor_parameters.hpp"

namespace policy_framework
{

namespace
{

/// Value of a received, finite entry.
std::optional<double> valid(const DataStore & store, size_t i)
{
  if (!store.received(i)) {return std::nullopt;}
  const double v = store.get(i);
  return std::isfinite(v) ? std::optional<double>(v) : std::nullopt;
}

}  // namespace

void PdEffortExecutor::configure(
  const PluginContext & ctx, const DataStore & commands, DataStore & state)
{
  const auto params = pd_effort_executor::ParamListener(ctx.node, ctx.name).get_params();
  auto names = params.joints;
  if (names.empty()) {
    for (const auto & n : commands.names()) {
      if (const auto joint = strip_suffix(n, "/position")) {names.push_back(*joint);}
    }
  }
  const auto & effort = params.effort_interface;
  const auto & kp = params.stiffness;
  const auto & kd = params.damping;
  const auto & limits = params.effort_limits;
  auto check_size = [&](const std::vector<double> & v, const std::string & what) {
      if (!v.empty() && v.size() != names.size()) {
        throw std::runtime_error(
                ctx.name + ": " + what + " has " + std::to_string(v.size()) + " values for " +
                std::to_string(names.size()) + " joints");
      }
    };
  check_size(kp, "stiffness");
  check_size(kd, "damping");
  check_size(limits, "effort_limits");
  const auto & fault_kd = params.fault_damping;
  check_size(fault_kd, "fault_damping");

  joints_.clear();
  interfaces_.clear();
  feedback_.clear();
  for (size_t i = 0; i < names.size(); ++i) {
    const auto & j = names[i];
    const auto ref_pos = commands.find(j + "/position");
    if (!ref_pos) {throw std::runtime_error(ctx.name + ": no command entry " + j + "/position");}
    Joint joint{*ref_pos, state.add(j + "/position"), state.add(j + "/velocity"),
      commands.find(j + "/velocity"), commands.find(j + "/stiffness"),
      commands.find(j + "/damping")};
    if ((!joint.kp_entry && kp.empty()) || (!joint.kd_entry && kd.empty())) {
      throw std::runtime_error(
              ctx.name + ": no gains for " + j +
              " (neither stiffness/damping command entries nor parameters)");
    }
    joint.has_kp = !kp.empty();
    joint.has_kd = !kd.empty();
    joint.kp = joint.has_kp ? kp[i] : 0.0;
    joint.kd = joint.has_kd ? kd[i] : 0.0;
    joint.limit = limits.empty() ? 0.0 : limits[i];
    joint.fault_kd = fault_kd.empty() ? 0.0 : fault_kd[i];
    joints_.push_back(joint);
    interfaces_.push_back(j + "/" + effort);
    feedback_.push_back(j + "/position");
    feedback_.push_back(j + "/velocity");
  }
}

double PdEffortExecutor::effort(
  const Joint & j, std::optional<double> ref, std::optional<double> ref_vel,
  const DataStore & commands, const DataStore & state)
{
  // Gains: the command entry once the policy wrote it, else the parameter if there is one.
  auto gain = [&](const std::optional<size_t> & entry, bool has_param, double param) {
      auto g = entry ? valid(commands, *entry) : std::nullopt;
      return g || !has_param ? g : std::optional<double>(param);
    };
  const auto q = valid(state, j.q);
  const auto dq = valid(state, j.dq);
  const auto kp = gain(j.kp_entry, j.has_kp, j.kp);
  const auto kd = gain(j.kd_entry, j.has_kd, j.kd);
  if (!ref || !q || !dq || !ref_vel || !kp || !kd) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double u = *kp * (*ref - *q) + *kd * (*ref_vel - *dq);
  return j.limit > 0.0 ? std::clamp(u, -j.limit, j.limit) : u;
}

void PdEffortExecutor::write(
  const DataStore & commands, const DataStore & state, std::vector<double> & values)
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & j = joints_[i];
    values[i] = effort(
      j, valid(commands, j.ref_pos),
      j.ref_vel ? valid(commands, *j.ref_vel) : std::optional<double>(0.0), commands, state);
  }
}

void PdEffortExecutor::hold(
  const DataStore & commands, const DataStore & state, std::vector<double> & values, bool start)
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    auto & j = joints_[i];
    if (start || std::isnan(j.hold_q)) {
      j.hold_q = valid(state, j.q).value_or(std::numeric_limits<double>::quiet_NaN());
    }
    values[i] = effort(
      j, std::isnan(j.hold_q) ? std::nullopt : std::optional<double>(j.hold_q), 0.0, commands,
      state);
  }
}

bool PdEffortExecutor::supports_fault_reaction(FaultReaction reaction) const
{
  if (reaction != FaultReaction::DAMP) {return true;}
  return !joints_.empty() && std::all_of(
    joints_.begin(), joints_.end(), [](const Joint & j) {return j.fault_kd > 0.0;});
}

void PdEffortExecutor::fault(
  const DataStore & commands, const DataStore & state, std::vector<double> & values,
  FaultReaction reaction, bool start)
{
  switch (reaction) {
    case FaultReaction::HOLD:
      hold(commands, state, values, start);
      // hold() leaves a joint it cannot compute unchanged; after a fault that would keep the
      // last effort of the policy applied.
      for (auto & v : values) {
        if (std::isnan(v)) {v = 0.0;}
      }
      return;
    case FaultReaction::DAMP:
      for (size_t i = 0; i < joints_.size(); ++i) {
        const auto & j = joints_[i];
        const auto dq = valid(state, j.dq);
        const double u = dq ? -j.fault_kd * *dq : 0.0;
        values[i] = j.limit > 0.0 ? std::clamp(u, -j.limit, j.limit) : u;
      }
      return;
    case FaultReaction::NEUTRAL_COMMANDS:  // never called with it: the host keeps calling write()
    case FaultReaction::SAFE:
    case FaultReaction::ERROR:
      safe_values(values);
      return;
  }
}

void ReferenceInterfaceExecutor::configure(
  const PluginContext & ctx, const DataStore & commands, DataStore &)
{
  const auto params = reference_interface_executor::ParamListener(ctx.node, ctx.name).get_params();
  const auto & prefix = params.prefix;
  const auto & kinds = params.interfaces;
  entries_.clear();
  interfaces_.clear();
  for (size_t i = 0; i < commands.size(); ++i) {
    const auto & name = commands.names()[i];
    const bool selected = std::any_of(
      kinds.begin(), kinds.end(), [&](const std::string & k) {return strip_suffix(name, "/" + k).has_value();});
    if (!selected) {continue;}
    entries_.push_back(i);
    interfaces_.push_back(prefix.empty() ? name : prefix + "/" + name);
  }
}

void ReferenceInterfaceExecutor::write(
  const DataStore & commands, const DataStore &, std::vector<double> & values)
{
  for (size_t i = 0; i < entries_.size(); ++i) {
    values[i] = commands.received(entries_[i]) ? commands.get(entries_[i]) :
      std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::PdEffortExecutor, policy_framework::InterfaceExecutor)
PLUGINLIB_EXPORT_CLASS(
  policy_framework::ReferenceInterfaceExecutor, policy_framework::InterfaceExecutor)
