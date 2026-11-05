// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__TOPIC_SOURCES_HPP_
#define POLICY_FRAMEWORK__TOPIC_SOURCES_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "policy_framework/latest_message.hpp"
#include "policy_framework/plugin_interfaces.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/**
 * @file topic_sources.hpp
 * @brief InputSource plugins fed by ROS topics.
 *
 * Every source reads the `<name>.topic` parameter (default `~/<name>`). Entry names mirror the
 * message field names and the ros2_control state interface naming, so a topic and a state
 * interface are interchangeable for the preprocessors.
 */

namespace policy_framework
{

/// Subscription + real-time safe hand-over of the latest message to the control cycle.
template<typename MsgT>
class TopicSource : public InputSource
{
public:
  void configure(const PluginContext & ctx) override
  {
    const auto topic = topic_parameter(ctx, "~/" + ctx.name);
    configure_source(ctx);
    topic_.subscribe(ctx.node, topic);
  }

protected:
  using Received = typename LatestMessage<MsgT>::Received;

  /// Hook for derived classes to read their own parameters.
  virtual void configure_source(const PluginContext &) {}

  /// @return The latest message (may be null) and when it arrived. Real-time safe.
  Received latest() {return topic_.get();}

private:
  LatestMessage<MsgT> topic_;
};

/**
 * @brief Fixed set of entries `<prefix>/<field>`, resolved against the store on first use.
 */
class FieldWriter
{
public:
  FieldWriter() = default;
  FieldWriter(const std::string & prefix, const std::vector<std::string> & fields);
  /// @return true when @p name is one of the entries.
  bool has(const std::string & name) const;
  /// Write values[i] to the i-th entry (if registered in @p store).
  void write(DataStore & store, const double * values, double stamp);

private:
  std::vector<std::string> names_;
  std::vector<std::optional<size_t>> indices_;  ///< Resolved on the first write().
};

/**
 * @brief sensor_msgs/JointState → `<joint>/position`, `<joint>/velocity`, `<joint>/effort`.
 *
 * Extra parameter `joints` (string array, default empty = any joint) restricts the joints this
 * source claims to provide.
 */
class JointStateSource : public TopicSource<sensor_msgs::msg::JointState>
{
public:
  bool provides(const std::string & name) const override;
  void read(DataStore & state, double now) override;

private:
  void configure_source(const PluginContext & ctx) override;

  std::vector<std::string> joints_;
  // Store index per (message joint, interface); rebuilt when the message joint order changes.
  std::vector<std::string> cached_names_;
  std::vector<std::optional<size_t>> pos_idx_, vel_idx_, eff_idx_;
};

/**
 * @brief sensor_msgs/Imu → `<sensor_name>/orientation.{x,y,z,w}`,
 *        `<sensor_name>/angular_velocity.{x,y,z}`, `<sensor_name>/linear_acceleration.{x,y,z}`.
 *
 * Extra parameter `sensor_name` (default: the source name), so that the entries can match the
 * names of the IMU state interfaces of the robot.
 */
class ImuSource : public TopicSource<sensor_msgs::msg::Imu>
{
public:
  bool provides(const std::string & name) const override {return fields_.has(name);}
  void read(DataStore & state, double now) override;

private:
  void configure_source(const PluginContext & ctx) override;
  FieldWriter fields_;
};

/**
 * @brief geometry_msgs/Twist → `<name>/linear.{x,y,z}`, `<name>/angular.{x,y,z}`.
 *
 * A velocity command that never arrived or is older than `timeout` seconds (parameter, default
 * 1.0, <= 0 disables it) is written as zero, so a policy never acts on a stale command and does
 * not wait for a command to start.
 */
class TwistSource : public TopicSource<geometry_msgs::msg::Twist>
{
public:
  bool provides(const std::string & name) const override {return fields_.has(name);}
  void read(DataStore & state, double now) override;

private:
  void configure_source(const PluginContext & ctx) override;
  FieldWriter fields_;
  double timeout_ = 1.0;
  bool timed_out_ = false;
  rclcpp::Logger logger_ = rclcpp::get_logger("twist_source");
};

/**
 * @brief nav_msgs/Odometry → `<name>/position.{x,y,z}`, `<name>/orientation.{x,y,z,w}`,
 *        `<name>/linear.{x,y,z}`, `<name>/angular.{x,y,z}` (twist, in the child frame).
 */
class OdometrySource : public TopicSource<nav_msgs::msg::Odometry>
{
public:
  bool provides(const std::string & name) const override {return fields_.has(name);}
  void read(DataStore & state, double now) override;

private:
  void configure_source(const PluginContext & ctx) override;
  FieldWriter fields_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__TOPIC_SOURCES_HPP_
