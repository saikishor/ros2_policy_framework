// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework_mjlab/motion_goal_term.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "policy_framework_mjlab/metadata.hpp"

namespace policy_framework_mjlab
{
namespace
{

constexpr size_t kRot6d = 6;  ///< A rotation as the first two columns of its matrix.

/// The rotation about z of @p orientation.
Eigen::Quaterniond heading(const Eigen::Quaterniond & orientation)
{
  const Eigen::Matrix3d r = orientation.toRotationMatrix();
  return Eigen::Quaterniond(Eigen::AngleAxisd(std::atan2(r(1, 0), r(0, 0)), Eigen::Vector3d::UnitZ()));
}

}  // namespace

MotionGoalTerm::MotionGoalTerm(std::string imu_name, std::string imu_frame)
: TopicTerm("~/motion_reference"), imu_name_(std::move(imu_name)),
  imu_frame_(std::move(imu_frame))
{
}

size_t MotionGoalTerm::configure_term(
  const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model)
{
  const auto & md = model.metadata;
  auto goal = joints(md, "goal_joint_names");
  goal_joints_ = std::move(goal.names);
  hold_positions_ = md.count("goal_hold_joint_pos") ?
    parse_doubles(md.at("goal_hold_joint_pos")) : std::move(goal.defaults);
  if (hold_positions_.size() != goal_joints_.size()) {
    throw std::runtime_error("goal_hold_joint_pos needs one value per goal joint");
  }
  frames_ = parse_counts(md, "goal_num_future_frames", 1).front();
  base_body_ = require(md, "goal_root_body");
  timeout_ = policy_framework::get_parameter<double>(ctx, "timeout", 0.5);
  gate_ = policy_framework::CommandGate(ctx);
  logger_ = ctx.node->get_logger().get_child("motion_goal");
  message_joints_.clear();
  columns_valid_ = false;
  streaming_ = false;
  return frames_ * (2 * goal_joints_.size() + kRot6d);
}

size_t MotionGoalTerm::configure(
  const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
  policy_framework::DataStore & state)
{
  const auto dim = TopicTerm::configure(ctx, model, state);
  base_.configure(ctx, state, imu_name_, imu_frame_, base_body_);
  return dim;
}

bool MotionGoalTerm::usable(const pal_motion_msgs::msg::MotionReference & msg)
{
  if (msg.joint_names != message_joints_) {
    // Only when the publisher changes its joints (in practice: on the first message), so the
    // allocations here stay out of the steady state.
    message_joints_ = msg.joint_names;
    columns_.clear();
    columns_valid_ = true;
    for (const auto & joint : goal_joints_) {
      const auto it = std::find(msg.joint_names.begin(), msg.joint_names.end(), joint);
      if (it == msg.joint_names.end()) {
        RCLCPP_WARN(logger_, "Ignoring the motion reference: it has no joint '%s'", joint.c_str());
        columns_valid_ = false;
        break;
      }
      columns_.push_back(static_cast<size_t>(it - msg.joint_names.begin()));
    }
  }
  bool well_formed = !msg.frames.empty() && msg.root_orientation.size() == msg.frames.size();
  for (size_t f = 0; well_formed && f < std::min(frames_, msg.frames.size()); ++f) {
    well_formed = msg.frames[f].positions.size() == msg.joint_names.size() &&
      msg.frames[f].velocities.size() == msg.joint_names.size();
  }
  if (!well_formed) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 1000, "Ignoring the motion reference: it needs frames with a "
      "position and a velocity per joint, and a root_orientation per frame");
  }
  return columns_valid_ && well_formed;
}

double MotionGoalTerm::compute(
  const policy_framework::DataStore & state, const policy_framework::TensorMap &, float * out)
{
  const Eigen::Quaterniond base = base_.orientation(state);
  const auto in = latest();
  const double t = now();
  const auto * reference =
    in.msg && t - in.receive_time < timeout_ && gate_.open(state) && usable(*in.msg) ?
    in.msg.get() : nullptr;

  auto root_of = [&](size_t frame) {
      const auto & q = reference->root_orientation[frame];
      return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
    };
  if (reference && !streaming_) {
    alignment_ = heading(base) * heading(root_of(0)).conjugate();
  }
  streaming_ = reference != nullptr;

  const size_t n = goal_joints_.size();
  const Eigen::Matrix3d world_to_base = base.toRotationMatrix().transpose();
  for (size_t k = 0; k < frames_; ++k) {
    float * frame = out + k * (2 * n + kRot6d);
    Eigen::Quaterniond root = heading(base);
    if (reference) {
      const size_t f = std::min(k, reference->frames.size() - 1);
      const auto & point = reference->frames[f];
      for (size_t j = 0; j < n; ++j) {
        frame[j] = static_cast<float>(point.positions[columns_[j]]);
        frame[n + j] = static_cast<float>(point.velocities[columns_[j]]);
      }
      root = alignment_ * root_of(f);
    } else {
      for (size_t j = 0; j < n; ++j) {
        frame[j] = static_cast<float>(hold_positions_[j]);
        frame[n + j] = 0.0F;
      }
    }
    // The root in the base frame, as the first two columns of its rotation matrix.
    const Eigen::Matrix3d r = world_to_base * root.toRotationMatrix();
    for (size_t i = 0; i < kRot6d; ++i) {
      frame[2 * n + i] = static_cast<float>(r(static_cast<Eigen::Index>(i % 3), static_cast<Eigen::Index>(i / 3)));
    }
  }
  return reference ? in.receive_time : t;
}

}  // namespace policy_framework_mjlab
