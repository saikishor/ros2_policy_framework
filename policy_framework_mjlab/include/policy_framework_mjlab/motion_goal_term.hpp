// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK_MJLAB__MOTION_GOAL_TERM_HPP_
#define POLICY_FRAMEWORK_MJLAB__MOTION_GOAL_TERM_HPP_

#include <string>
#include <vector>

#include "pal_motion_msgs/msg/motion_reference.hpp"
#include "policy_framework/observation_term.hpp"
#include "policy_framework_mjlab/base_from_imu.hpp"

namespace policy_framework_mjlab
{

/**
 * @brief The `motion_goal` term of pal_mjlab supermimic policies: the reference motion the
 *        policy tracks, from a pal_motion_msgs/MotionReference topic.
 *
 * Layout: `goal_num_future_frames` frames, each the positions of `goal_joint_names`, their
 * velocities, and the orientation of the reference root in the base (`goal_root_body`) frame
 * as the first two columns of its rotation matrix. A message with fewer frames has its last
 * one repeated, so a live source can send a single frame.
 *
 * When a stream starts (first reference after none, or after reset()) it is turned about z so
 * that its first root has the heading of the robot, and keeps that alignment.
 *
 * Without a usable reference the goal is a still stand at the heading of the robot:
 * `goal_hold_joint_pos` (else the default positions of the goal joints), zero velocities. That
 * is the case before the first message, when the last one is older than `timeout`, when it is
 * malformed (logged), and while the host's `safety_entry` is 0 (policy_framework::CommandGate).
 * So the term is always ready once the base is: a policy does not wait for a reference.
 *
 * The base orientation comes from the IMU through the kinematics (BaseFromImu), so it needs
 * `kinematics.enabled` and the preprocessor's `imu_frame`.
 *
 * A policy_framework::CommandTerm: parameters (under `<policy>.terms.motion_goal.`) `topic`
 * (default `~/motion_reference`) and `timeout` (seconds, default 0.5, above zero); stamped
 * with the header stamp of the reference, else now.
 *
 * Stamp: the receive time of the reference; the current time for a hold.
 */
class MotionGoalTerm : public policy_framework::CommandTerm<pal_motion_msgs::msg::MotionReference>
{
public:
  /// @param imu_name Prefix of the IMU state entries. @param imu_frame URDF link of the IMU.
  MotionGoalTerm(std::string imu_name, std::string imu_frame);

  /// @throws std::runtime_error on missing metadata, or as BaseFromImu::configure().
  size_t configure(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
    policy_framework::DataStore & state) override;
  bool ready(const policy_framework::DataStore & state) override {return base_.ready(state);}
  /// The next reference starts a new stream.
  void reset() override {streaming_ = false;}

protected:
  size_t configure_command(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model)
  override;
  /// @return true when @p msg can be used; updates columns_ when its joints changed.
  bool usable(const pal_motion_msgs::msg::MotionReference & msg) override;
  void write_command(
    const pal_motion_msgs::msg::MotionReference & msg, const policy_framework::DataStore & state,
    float * out) override;
  void write_neutral(const policy_framework::DataStore & state, float * out) override;

private:
  /// The goal frames from @p reference, or the still stand without one (null).
  void write(
    const pal_motion_msgs::msg::MotionReference * reference,
    const policy_framework::DataStore & state, float * out);

  std::string imu_name_, imu_frame_, base_body_;
  BaseFromImu base_;
  rclcpp::Logger logger_ = rclcpp::get_logger("motion_goal");
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::vector<std::string> goal_joints_;
  std::vector<double> hold_positions_;
  size_t frames_ = 0;
  // Index in the message of each goal joint, for the message joints message_joints_.
  std::vector<std::string> message_joints_;
  std::vector<size_t> columns_;
  bool columns_valid_ = false;
  bool streaming_ = false;
  Eigen::Quaterniond alignment_ = Eigen::Quaterniond::Identity();  ///< Of the current stream.
};

}  // namespace policy_framework_mjlab

#endif  // POLICY_FRAMEWORK_MJLAB__MOTION_GOAL_TERM_HPP_
