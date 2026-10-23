// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__KINEMATICS_HPP_
#define POLICY_FRAMEWORK__KINEMATICS_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "policy_framework/data_store.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace policy_framework
{

/**
 * @brief Kinematic model of one chain of the robot, shared by every plugin of a runner and
 *        updated once per policy step from the state store, so observation terms only read it.
 *
 * Wraps a kinematics_interface plugin (e.g. kinematics_interface_kdl/KinematicsInterfaceKDL).
 * Parameters, under `kinematics.`:
 * - `plugin_name`: plugin class; empty (default) disables kinematics.
 * - `base`, `tip`: the chain; `base` must be an ancestor of `tip` in the URDF.
 * - `alpha`: damping of the plugin's Jacobian inverse.
 * - `imu_sensor`, `imu_link` (optional, together): state prefix of the IMU orientation
 *   (`<imu_sensor>/orientation.{x,y,z,w}`, its orientation in the world) and the link of the IMU
 *   frame, on the chain. Enables world_orientation().
 *
 * It registers `<joint>/position` of every movable chain joint (and the IMU orientation) in the
 * state store, so a controller claims them as state interfaces.
 *
 * Usage: plugins call frame() while configuring; the runner calls update() at the start of every
 * step; plugins read transform() / world_orientation() during the step. Not thread-safe: use it
 * only from the thread running the policy step (not from interface executors).
 */
class Kinematics
{
public:
  /**
   * @brief Load and initialize the plugin from the `kinematics.` parameters of @p node.
   * @param robot_description URDF of the robot.
   * @param state Store in which the chain joint positions (and IMU orientation) are registered.
   * @return null when `kinematics.plugin_name` is empty.
   * @throws std::runtime_error on an invalid configuration (no URDF, unknown links, a plugin
   *         that fails to load or initialize, an IMU link outside the chain).
   */
  static std::shared_ptr<Kinematics> create(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & robot_description, DataStore & state);

  ~Kinematics();

  /**
   * @brief Request the transform of @p link, computed from now on in every update(). Call while
   *        configuring (allocates). Idempotent.
   * @return Handle for transform() and world_orientation().
   * @throws std::runtime_error when @p link is not on the chain.
   */
  size_t frame(const std::string & link);

  /// Recompute the requested frames from @p state (no allocation). Called by the runner.
  void update(const DataStore & state);

  /// @return true when the last update() had every input (joints, IMU) received.
  bool ready() const {return ready_;}
  /// @return Oldest stamp of the inputs of the last update() (seconds).
  double stamp() const {return stamp_;}

  /// @return Pose of the frame in the `base` frame, as of the last update().
  const Eigen::Isometry3d & transform(size_t frame) const {return transforms_[frame];}
  /**
   * @return Orientation of the frame in the world, from the IMU orientation and the chain.
   * @throws std::logic_error when no IMU is configured.
   */
  Eigen::Quaterniond world_orientation(size_t frame) const;

  /// @return Movable chain joints, in the order of the plugin's joint vector.
  const std::vector<std::string> & joint_names() const {return joint_names_;}

private:
  Kinematics() = default;

  // Loader before the plugin: the library must outlive the instance.
  std::unique_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> loader_;
  std::shared_ptr<kinematics_interface::KinematicsInterface> plugin_;
  std::vector<std::string> joint_names_;
  std::vector<size_t> joint_idx_;
  std::vector<size_t> imu_idx_;  ///< orientation x, y, z, w; empty without IMU.
  size_t imu_frame_ = 0;
  Eigen::VectorXd q_;
  std::vector<std::string> links_;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> transforms_;
  Eigen::Quaterniond imu_orientation_ = Eigen::Quaterniond::Identity();
  bool ready_ = false;
  double stamp_ = 0.0;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__KINEMATICS_HPP_
