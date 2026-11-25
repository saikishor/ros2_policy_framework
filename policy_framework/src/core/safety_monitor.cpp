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

#include "policy_framework/core/safety_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <fmt/format.h>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace policy_framework
{

std::string SafetyMonitor::Violation::describe() const
{
  switch (check) {
    case Check::TILT:
      return fmt::format("tilt of {} rad above safety.max_tilt", value);
    case Check::JOINT_POSITION:
      return fmt::format(
        "position {} of joint {} outside its limits minus safety.position_margin", value, joint);
    case Check::INPUT_AGE:
      return fmt::format("observation {} s old, above safety.max_input_age", value);
  }
  return "unknown check";
}

void SafetyMonitor::configure(const Config & config, const DataStore & commands, DataStore & state)
{
  config_ = config;
  orientation_.clear();
  positions_.clear();
  if (config.max_tilt > 0.0) {
    if (config.imu.empty()) {
      throw std::invalid_argument("safety.max_tilt needs safety.imu");
    }
    for (const auto * axis : {"x", "y", "z", "w"}) {
      orientation_.push_back(state.add(config.imu + "/orientation." + axis));
    }
  }
  const std::string suffix = "/position";
  for (const auto & name : commands.names()) {
    if (name.size() <= suffix.size() ||
      name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0)
    {
      continue;
    }
    const auto joint = name.substr(0, name.size() - suffix.size());
    if (const auto it = config.joint_limits.find(joint); it != config.joint_limits.end()) {
      positions_.push_back(
        {joint, state.add(joint + "/position"), it->second.first + config.position_margin,
          it->second.second - config.position_margin});
    }
  }
}

std::optional<SafetyMonitor::Violation> SafetyMonitor::check(
  const DataStore & state, double observation_stamp, double now, const Checks & skipped) const
{
  if (!skipped[bit(Check::TILT)] && !orientation_.empty() && state.all_received(orientation_)) {
    const double x = state.get(orientation_[0]);
    const double y = state.get(orientation_[1]);
    const double z = state.get(orientation_[2]);
    const double w = state.get(orientation_[3]);
    const double norm2 = x * x + y * y + z * z + w * w;
    if (norm2 > 0.0) {
      // z component of the IMU z axis in the world: the cosine of the tilt.
      const double c = std::clamp(1.0 - 2.0 * (x * x + y * y) / norm2, -1.0, 1.0);
      const double tilt = std::acos(c);
      if (tilt > config_.max_tilt) {return Violation{Check::TILT, tilt, ""};}
    }
  }
  for (const auto & j : positions_) {
    if (skipped[bit(Check::JOINT_POSITION)]) {break;}
    if (!state.received(j.index)) {continue;}
    const double q = state.get(j.index);
    if (q < j.lower || q > j.upper) {return Violation{Check::JOINT_POSITION, q, j.name};}
  }
  if (!skipped[bit(Check::INPUT_AGE)] && config_.max_input_age > 0.0 &&
    observation_stamp > 0.0 && now - observation_stamp > config_.max_input_age)
  {
    return Violation{Check::INPUT_AGE, now - observation_stamp, ""};
  }
  return std::nullopt;
}

std::map<std::string, std::pair<double, double>> SafetyMonitor::joint_limits(
  const std::string & urdf)
{
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(urdf, model);
  std::map<std::string, std::pair<double, double>> limits;
  for (pinocchio::JointIndex j = 1; j < model.joints.size(); ++j) {
    const auto & joint = model.joints[j];
    if (joint.nq() != 1) {continue;}  // continuous (cos, sin) or multi-dof joints
    const double lower = model.lowerPositionLimit[joint.idx_q()];
    const double upper = model.upperPositionLimit[joint.idx_q()];
    if (std::isfinite(lower) && std::isfinite(upper) && lower < upper) {
      limits[model.names[j]] = {lower, upper};
    }
  }
  return limits;
}

}  // namespace policy_framework
