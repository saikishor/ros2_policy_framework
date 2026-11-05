// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/topic_sources.hpp"

#include "entry_name.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/imu_source_parameters.hpp"
#include "policy_framework/joint_state_source_parameters.hpp"
#include "policy_framework/topic_parameters.hpp"
#include "policy_framework/twist_source_parameters.hpp"

namespace policy_framework
{

std::string topic_parameter(const PluginContext & ctx, const std::string & default_topic)
{
  const auto topic = topic_parameters::ParamListener(ctx.node, ctx.name).get_params().topic;
  return topic.empty() ? default_topic : topic;
}
namespace
{

/// Header stamp in seconds, or @p fallback when the publisher left it empty.
double stamp_or(const builtin_interfaces::msg::Time & t, double fallback)
{
  return (t.sec == 0 && t.nanosec == 0) ? fallback : t.sec + t.nanosec * 1e-9;
}

}  // namespace

FieldWriter::FieldWriter(const std::string & prefix, const std::vector<std::string> & fields)
{
  for (const auto & f : fields) {names_.push_back(prefix + "/" + f);}
}

bool FieldWriter::has(const std::string & name) const
{
  return std::find(names_.begin(), names_.end(), name) != names_.end();
}

void FieldWriter::write(DataStore & store, const double * values, double stamp)
{
  if (indices_.size() != names_.size()) {
    // First call only (allocates once): the store is complete once the control cycle runs.
    for (const auto & n : names_) {indices_.push_back(store.find(n));}
  }
  for (size_t i = 0; i < indices_.size(); ++i) {
    if (indices_[i]) {store.set(*indices_[i], values[i], stamp);}
  }
}

// ---------------------------------------------------------------------------------- JointState

void JointStateSource::configure_source(const PluginContext & ctx)
{
  joints_ = joint_state_source::ParamListener(ctx.node, ctx.name).get_params().joints;
}

bool JointStateSource::provides(const std::string & name) const
{
  for (const auto * suffix : {"/position", "/velocity", "/effort"}) {
    const auto joint = strip_suffix(name, suffix);
    if (!joint) {continue;}
    return joints_.empty() || std::find(joints_.begin(), joints_.end(), *joint) != joints_.end();
  }
  return false;
}

void JointStateSource::read(DataStore & state, double now)
{
  const auto in = latest();
  if (!in.msg) {return;}
  const auto & msg = *in.msg;
  if (msg.name != cached_names_) {
    // Only when the publisher changes the joint order (in practice: on the first message).
    cached_names_ = msg.name;
    pos_idx_.clear();
    vel_idx_.clear();
    eff_idx_.clear();
    for (const auto & j : msg.name) {
      const bool ok = joints_.empty() || std::find(joints_.begin(), joints_.end(), j) !=
        joints_.end();
      pos_idx_.push_back(ok ? state.find(j + "/position") : std::nullopt);
      vel_idx_.push_back(ok ? state.find(j + "/velocity") : std::nullopt);
      eff_idx_.push_back(ok ? state.find(j + "/effort") : std::nullopt);
    }
  }
  const double stamp = stamp_or(msg.header.stamp, now);
  for (size_t i = 0; i < msg.name.size(); ++i) {
    if (pos_idx_[i] && i < msg.position.size()) {state.set(*pos_idx_[i], msg.position[i], stamp);}
    if (vel_idx_[i] && i < msg.velocity.size()) {state.set(*vel_idx_[i], msg.velocity[i], stamp);}
    if (eff_idx_[i] && i < msg.effort.size()) {state.set(*eff_idx_[i], msg.effort[i], stamp);}
  }
}

// ----------------------------------------------------------------------------------------- Imu

void ImuSource::configure_source(const PluginContext & ctx)
{
  const auto sensor = imu_source::ParamListener(ctx.node, ctx.name).get_params().sensor_name;
  fields_ = FieldWriter(
    sensor.empty() ? ctx.name : sensor,
    {"orientation.x", "orientation.y", "orientation.z", "orientation.w",
      "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
      "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"});
}

void ImuSource::read(DataStore & state, double now)
{
  const auto in = latest();
  if (!in.msg) {return;}
  const auto & m = *in.msg;
  const double v[] = {
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w,
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z};
  fields_.write(state, v, stamp_or(m.header.stamp, now));
}

// --------------------------------------------------------------------------------------- Twist

void TwistSource::configure_source(const PluginContext & ctx)
{
  timeout_ = twist_source::ParamListener(ctx.node, ctx.name).get_params().timeout;
  logger_ = ctx.node->get_logger().get_child(ctx.name);
  fields_ = FieldWriter(
    ctx.name, {"linear.x", "linear.y", "linear.z", "angular.x", "angular.y", "angular.z"});
}

void TwistSource::read(DataStore & state, double now)
{
  const auto in = latest();
  const bool stale = in.msg && timeout_ > 0.0 && now - in.receive_time > timeout_;
  if (stale != timed_out_) {
    // Edge triggered: logs once per transition, not every cycle.
    RCLCPP_WARN_EXPRESSION(logger_, stale, "No velocity command for %.2f s, using zero", timeout_);
    RCLCPP_INFO_EXPRESSION(logger_, !stale, "Velocity command resumed");
    timed_out_ = stale;
  }
  if (!in.msg || stale) {
    const double zero[6] = {};
    fields_.write(state, zero, now);
    return;
  }
  const auto & m = *in.msg;
  const double v[] = {m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z};
  fields_.write(state, v, in.receive_time);
}

// ------------------------------------------------------------------------------------ Odometry

void OdometrySource::configure_source(const PluginContext & ctx)
{
  fields_ = FieldWriter(
    ctx.name, {"position.x", "position.y", "position.z",
      "orientation.x", "orientation.y", "orientation.z", "orientation.w",
      "linear.x", "linear.y", "linear.z", "angular.x", "angular.y", "angular.z"});
}

void OdometrySource::read(DataStore & state, double now)
{
  const auto in = latest();
  if (!in.msg) {return;}
  const auto & p = in.msg->pose.pose;
  const auto & t = in.msg->twist.twist;
  const double v[] = {
    p.position.x, p.position.y, p.position.z,
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
    t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z};
  fields_.write(state, v, stamp_or(in.msg->header.stamp, now));
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::JointStateSource, policy_framework::InputSource)
PLUGINLIB_EXPORT_CLASS(policy_framework::ImuSource, policy_framework::InputSource)
PLUGINLIB_EXPORT_CLASS(policy_framework::TwistSource, policy_framework::InputSource)
PLUGINLIB_EXPORT_CLASS(policy_framework::OdometrySource, policy_framework::InputSource)
