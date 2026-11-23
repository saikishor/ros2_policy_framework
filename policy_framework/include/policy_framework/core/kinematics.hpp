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

#ifndef POLICY_FRAMEWORK__CORE__KINEMATICS_HPP_
#define POLICY_FRAMEWORK__CORE__KINEMATICS_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "realtime_tools/mutex.hpp"

namespace policy_framework
{

/**
 * @brief Kinematic model of the whole robot (fixed base at the URDF root link), computed with
 *        Pinocchio.
 *
 * One thread (the control loop) writes the joint positions and velocities with update(), which
 * computes the forward kinematics of every frame; any thread reads transforms and velocities.
 * Both sides share a priority-inheritance recursive mutex (realtime_tools):
 * - update() only try_lock()s: it never waits, and returns false (update skipped, the previous
 *   state kept) while a reader holds the lock;
 * - readers lock (blocking) for the time of a copy only, never for the forward kinematics; a
 *   reader in the control loop thus waits at most for one update().
 * Readers return copies. Use with_lock() for several values from the same update.
 */
class Kinematics
{
public:
  /// Frame in which velocity() expresses a frame's velocity.
  enum class Reference
  {
    LOCAL,        ///< The frame's own axes.
    ROOT_ALIGNED  ///< At the frame's origin, with the axes of the root link.
  };

  /**
   * @brief Build the model from @p robot_description.
   * @param joints Joints given to update(); every other movable joint is locked at zero (e.g.
   *        joints with no state interface). Empty: every movable joint.
   * @throws std::runtime_error on an unparsable URDF, an unknown joint, or a selected joint
   *         that is not revolute, continuous or prismatic.
   */
  explicit Kinematics(
    const std::string & robot_description, const std::vector<std::string> & joints = {});
  ~Kinematics();
  Kinematics(const Kinematics &) = delete;
  Kinematics & operator=(const Kinematics &) = delete;

  /// @return The joints of update()'s vectors, in their order (the model's order).
  const std::vector<std::string> & joint_names() const;

  /**
   * @return Index of the link @p link for the readers. Not real-time (string lookup): call it
   *         while configuring.
   * @throws std::runtime_error when the URDF has no such link.
   */
  size_t frame(const std::string & link) const;

  /**
   * @brief Set the joint state and compute the forward kinematics (positions and velocities of
   *        every frame). Real-time safe: no allocation, never blocks.
   * @param q Positions (rad, m), in joint_names() order.
   * @param v Velocities, in joint_names() order.
   * @param stamp Time of the state (seconds), returned by stamp().
   * @return false when a reader holds the lock: nothing changed.
   * @throws std::invalid_argument when a size differs from joint_names().
   */
  bool update(const Eigen::VectorXd & q, const Eigen::VectorXd & v, double stamp);

  /// @return true once an update() succeeded.
  bool ready() const;
  /// @return Stamp of the last successful update().
  double stamp() const;

  /// @return Pose of @p frame in the root link.
  Eigen::Isometry3d transform(size_t frame) const;
  /// @return Pose of @p frame in @p reference.
  Eigen::Isometry3d transform(size_t frame, size_t reference) const;
  /// @return Spatial velocity of @p frame: linear (m/s) then angular (rad/s).
  Eigen::Matrix<double, 6, 1> velocity(size_t frame, Reference reference) const;
  /**
   * @return Orientation in the world of @p frame, given the measured orientation @p imu of the
   *         IMU frame @p imu_frame in the world.
   */
  Eigen::Quaterniond world_orientation(
    size_t frame, size_t imu_frame, const Eigen::Quaterniond & imu) const;

  /**
   * @return Angular velocity in the world of @p frame, in its own axes (rad/s), given the
   *         angular velocity @p imu_angular_velocity measured by a gyroscope in the frame
   *         @p imu_frame (in the IMU axes): what that gyroscope would measure mounted on
   *         @p frame, i.e. without the motion of the joints in between.
   */
  Eigen::Vector3d world_angular_velocity(
    size_t frame, size_t imu_frame, const Eigen::Vector3d & imu_angular_velocity) const;

  /// Call @p f(*this) under the lock: its reads all come from the same update().
  template<typename F>
  void with_lock(F && f) const
  {
    std::lock_guard<realtime_tools::prio_inherit_recursive_mutex> lock(mutex_);
    f(*this);
  }

private:
  struct Model;  // Pinocchio, kept out of this header
  std::unique_ptr<Model> model_;
  mutable realtime_tools::prio_inherit_recursive_mutex mutex_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__CORE__KINEMATICS_HPP_
