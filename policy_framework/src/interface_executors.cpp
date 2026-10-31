// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/interface_executors.hpp"

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

const std::string kPosition = "/position";

bool ends_with(const std::string & s, const std::string & suffix)
{
  return s.size() > suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

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
      if (ends_with(n, kPosition)) {names.push_back(n.substr(0, n.size() - kPosition.size()));}
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

  joints_.clear();
  interfaces_.clear();
  feedback_.clear();
  for (size_t i = 0; i < names.size(); ++i) {
    const auto & j = names[i];
    const auto ref_pos = commands.find(j + kPosition);
    if (!ref_pos) {throw std::runtime_error(ctx.name + ": no command entry " + j + kPosition);}
    Joint joint{*ref_pos, state.add(j + "/position"), state.add(j + "/velocity"),
      commands.find(j + "/velocity"), commands.find(j + "/stiffness"),
      commands.find(j + "/damping")};
    if ((!joint.kp_entry && kp.empty()) || (!joint.kd_entry && kd.empty())) {
      throw std::runtime_error(
              ctx.name + ": no gains for " + j +
              " (neither stiffness/damping command entries nor parameters)");
    }
    joint.kp = kp.empty() ? 0.0 : kp[i];
    joint.kd = kd.empty() ? 0.0 : kd[i];
    joint.limit = limits.empty() ? 0.0 : limits[i];
    joints_.push_back(joint);
    interfaces_.push_back(j + "/" + effort);
    feedback_.push_back(j + "/position");
    feedback_.push_back(j + "/velocity");
  }
}

void PdEffortExecutor::write(
  const DataStore & commands, const DataStore & state, std::vector<double> & values)
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & j = joints_[i];
    const auto ref = valid(commands, j.ref_pos);
    const auto q = valid(state, j.q);
    const auto dq = valid(state, j.dq);
    const auto ref_vel = j.ref_vel ? valid(commands, *j.ref_vel) : std::optional<double>(0.0);
    const auto kp = j.kp_entry ? valid(commands, *j.kp_entry) : std::optional<double>(j.kp);
    const auto kd = j.kd_entry ? valid(commands, *j.kd_entry) : std::optional<double>(j.kd);
    if (!ref || !q || !dq || !ref_vel || !kp || !kd) {
      values[i] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    double u = *kp * (*ref - *q) + *kd * (*ref_vel - *dq);
    if (j.limit > 0.0) {u = std::clamp(u, -j.limit, j.limit);}
    values[i] = u;
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
      kinds.begin(), kinds.end(), [&](const std::string & k) {return ends_with(name, "/" + k);});
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
