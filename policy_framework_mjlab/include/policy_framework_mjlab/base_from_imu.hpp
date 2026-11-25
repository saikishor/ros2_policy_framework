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

#ifndef POLICY_FRAMEWORK_MJLAB__BASE_FROM_IMU_HPP_
#define POLICY_FRAMEWORK_MJLAB__BASE_FROM_IMU_HPP_

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <fmt/format.h>

#include "policy_framework/core/kinematics.hpp"
#include "policy_framework/plugin_interfaces.hpp"

namespace policy_framework_mjlab
{

/**
 * @brief Orientation and angular velocity of the base of a policy (a URDF link, pal_mjlab's
 *        `goal_root_body`) from an IMU mounted on another link: what that IMU would measure on
 *        the base, using the kinematics for the joints in between.
 *
 * Reads the state entries `<imu_name>/orientation.{w,x,y,z}` and
 * `<imu_name>/angular_velocity.{x,y,z}`.
 */
class BaseFromImu
{
public:
  /**
   * @param imu_frame URDF link of the IMU.
   * @throws std::runtime_error without kinematics (`kinematics.enabled`), with an empty
   *         @p imu_frame, or when a link is not in the URDF.
   */
  void configure(
    const policy_framework::PluginContext & ctx, policy_framework::DataStore & state,
    const std::string & imu_name, const std::string & imu_frame, const std::string & base)
  {
    if (!ctx.kinematics) {
      throw std::runtime_error(fmt::format("The base '{}' needs kinematics.enabled", base));
    }
    if (imu_frame.empty()) {
      throw std::runtime_error(fmt::format("The base '{}' needs the 'imu_frame' parameter", base));
    }
    kinematics_ = ctx.kinematics;
    base_ = kinematics_->frame(base);
    imu_ = kinematics_->frame(imu_frame);
    idx_.clear();
    for (const auto * field : {"orientation.w", "orientation.x", "orientation.y", "orientation.z",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z"})
    {
      idx_.push_back(state.add(imu_name + "/" + field));
    }
  }

  /// @return true once the IMU and the joint state of the kinematics were received.
  bool ready(const policy_framework::DataStore & state) const
  {
    return state.all_received(idx_) && kinematics_->ready();
  }

  /// @return Orientation of the base in the world. Real-time safe, like the methods below.
  Eigen::Quaterniond orientation(const policy_framework::DataStore & state) const
  {
    const Eigen::Quaterniond imu(
      state.get(idx_[0]), state.get(idx_[1]), state.get(idx_[2]), state.get(idx_[3]));
    return kinematics_->world_orientation(base_, imu_, imu.normalized());
  }

  /// @return Angular velocity of the base in the world, in the base axes.
  Eigen::Vector3d angular_velocity(const policy_framework::DataStore & state) const
  {
    return kinematics_->world_angular_velocity(
      base_, imu_, {state.get(idx_[4]), state.get(idx_[5]), state.get(idx_[6])});
  }

  /// @return Oldest stamp of the IMU entries and of the joint state of the kinematics.
  double stamp(const policy_framework::DataStore & state) const
  {
    double t = kinematics_->stamp();
    for (const auto i : idx_) {t = std::min(t, state.stamp(i));}
    return t;
  }

private:
  std::shared_ptr<policy_framework::Kinematics> kinematics_;
  size_t base_ = 0;
  size_t imu_ = 0;
  std::vector<size_t> idx_;
};

}  // namespace policy_framework_mjlab

#endif  // POLICY_FRAMEWORK_MJLAB__BASE_FROM_IMU_HPP_
