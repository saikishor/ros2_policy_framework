// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "policy_framework/topic_sources.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::DataStore;
using policy_framework::PluginContext;

namespace
{

class TopicSourcesTest : public ::testing::Test
{
protected:
  void make_node(const std::vector<rclcpp::Parameter> & params)
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(params);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_sources", opts);
    pub_node = std::make_shared<rclcpp::Node>("test_pub");
    exec.add_node(node->get_node_base_interface());
    exec.add_node(pub_node);
  }

  template<typename MsgT>
  void publish(const std::string & topic, const MsgT & msg)
  {
    auto pub = pub_node->create_publisher<MsgT>(topic, rclcpp::SystemDefaultsQoS());
    // Wait for discovery, then give the executor time to deliver.
    for (int i = 0; i < 100 && pub->get_subscription_count() == 0; ++i) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    pub->publish(msg);
    for (int i = 0; i < 20; ++i) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  double value(const std::string & name) const {return store.get(store.index(name));}

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr pub_node;
  rclcpp::executors::SingleThreadedExecutor exec;
  DataStore store;
};

}  // namespace

TEST_F(TopicSourcesTest, JointStateWritesRegisteredJointsOnly)
{
  make_node({{"js.topic", "/test_joint_states"}});
  policy_framework::JointStateSource src;
  src.configure({node, "js"});
  EXPECT_TRUE(src.provides("j1/position"));
  EXPECT_TRUE(src.provides("j2/effort"));
  EXPECT_FALSE(src.provides("imu/orientation.x"));

  const auto p1 = store.add("j1/position");
  const auto v2 = store.add("j2/velocity");
  src.read(store, 1.0);
  EXPECT_FALSE(store.received(p1));  // nothing published yet

  sensor_msgs::msg::JointState msg;
  msg.name = {"j2", "j1", "j3"};
  msg.position = {2.0, 1.0, 3.0};
  msg.velocity = {20.0, 10.0, 30.0};
  publish("/test_joint_states", msg);
  src.read(store, 1.0);
  EXPECT_DOUBLE_EQ(store.get(p1), 1.0);
  EXPECT_DOUBLE_EQ(store.get(v2), 20.0);
  EXPECT_EQ(store.size(), 2u);  // j3 was not registered and is ignored
}

TEST_F(TopicSourcesTest, JointStateJointsParameterRestrictsProvides)
{
  make_node({{"js.topic", "/test_joint_states2"}, {"js.joints", std::vector<std::string>{"j1"}}});
  policy_framework::JointStateSource src;
  src.configure({node, "js"});
  EXPECT_TRUE(src.provides("j1/position"));
  EXPECT_FALSE(src.provides("j2/position"));
}

TEST_F(TopicSourcesTest, ImuWritesStateInterfaceStyleNames)
{
  make_node({{"imu.topic", "/test_imu"}, {"imu.sensor_name", "torso_imu"}});
  policy_framework::ImuSource src;
  src.configure({node, "imu"});
  EXPECT_TRUE(src.provides("torso_imu/orientation.w"));
  EXPECT_FALSE(src.provides("imu/orientation.w"));
  for (const auto * n : {"torso_imu/orientation.w", "torso_imu/angular_velocity.z",
      "torso_imu/linear_acceleration.x"})
  {
    store.add(n);
  }
  sensor_msgs::msg::Imu msg;
  msg.orientation.w = 0.5;
  msg.angular_velocity.z = 3.0;
  msg.linear_acceleration.x = 9.0;
  publish("/test_imu", msg);
  src.read(store, 0.0);
  EXPECT_DOUBLE_EQ(value("torso_imu/orientation.w"), 0.5);
  EXPECT_DOUBLE_EQ(value("torso_imu/angular_velocity.z"), 3.0);
  EXPECT_DOUBLE_EQ(value("torso_imu/linear_acceleration.x"), 9.0);
}

TEST_F(TopicSourcesTest, TwistIsZeroBeforeFirstMessageAndAfterTimeout)
{
  make_node({{"cmd_vel.topic", "/test_cmd_vel"}, {"cmd_vel.timeout", 0.5}});
  policy_framework::TwistSource src;
  src.configure({node, "cmd_vel"});
  const auto vx = store.add("cmd_vel/linear.x");
  const auto wz = store.add("cmd_vel/angular.z");

  src.read(store, 100.0);
  EXPECT_TRUE(store.received(vx));  // a missing velocity command means "stand still"
  EXPECT_DOUBLE_EQ(store.get(vx), 0.0);

  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.3;
  msg.angular.z = -0.2;
  publish("/test_cmd_vel", msg);
  const double now = node->get_clock()->now().seconds();
  src.read(store, now);
  EXPECT_DOUBLE_EQ(store.get(vx), 0.3);
  EXPECT_DOUBLE_EQ(store.get(wz), -0.2);
  // A Twist has no header: the command in effect is stamped now, not with its receive time.
  src.read(store, now + 0.1);
  EXPECT_DOUBLE_EQ(store.get(vx), 0.3);
  EXPECT_DOUBLE_EQ(store.stamp(vx), now + 0.1);

  src.read(store, now + 1.0);  // older than the 0.5 s timeout
  EXPECT_DOUBLE_EQ(store.get(vx), 0.0);
  EXPECT_DOUBLE_EQ(store.get(wz), 0.0);
}

TEST_F(TopicSourcesTest, TwistRejectsADisabledTimeout)
{
  make_node({{"cmd_vel.timeout", 0.0}});
  policy_framework::TwistSource src;
  EXPECT_ANY_THROW(src.configure({node, "cmd_vel"}));
}

TEST_F(TopicSourcesTest, TwistIsZeroWhileTheSafetyEntryIsNotSet)
{
  make_node({{"cmd_vel.topic", "/test_cmd_vel_safety"}});
  policy_framework::TwistSource src;
  src.configure({node, "cmd_vel", nullptr, "safety/safe"});
  const auto vx = store.add("cmd_vel/linear.x");
  const auto safe = store.add("safety/safe");
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.3;
  publish("/test_cmd_vel_safety", msg);
  const double now = node->get_clock()->now().seconds();

  src.read(store, now);  // safety not known yet: not safe
  EXPECT_DOUBLE_EQ(store.get(vx), 0.0);
  store.set(safe, 1.0, now);
  src.read(store, now);
  EXPECT_DOUBLE_EQ(store.get(vx), 0.3);
  store.set(safe, 0.0, now);
  src.read(store, now);
  EXPECT_DOUBLE_EQ(store.get(vx), 0.0);
}

TEST_F(TopicSourcesTest, TwistIsZeroWhileTheHostSetsNeutralCommands)
{
  make_node({{"cmd_vel.topic", "/test_cmd_vel_neutral"}});
  policy_framework::TwistSource src;
  src.configure({node, "cmd_vel"});  // no safety entry: only the host's neutral commands gate
  const auto vx = store.add("cmd_vel/linear.x");
  const auto neutral = store.add(policy_framework::kNeutralCommandsEntry);
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.3;
  publish("/test_cmd_vel_neutral", msg);
  const double now = node->get_clock()->now().seconds();

  src.read(store, now);  // never set: commands accepted
  EXPECT_DOUBLE_EQ(store.get(vx), 0.3);
  store.set(neutral, 1.0, now);
  src.read(store, now);
  EXPECT_DOUBLE_EQ(store.get(vx), 0.0);
  store.set(neutral, 0.0, now);  // recovers by itself
  src.read(store, now);
  EXPECT_DOUBLE_EQ(store.get(vx), 0.3);
}

TEST_F(TopicSourcesTest, OdometryWritesPoseAndTwist)
{
  make_node({{"odom.topic", "/test_odom"}});
  policy_framework::OdometrySource src;
  src.configure({node, "odom"});
  const auto vx = store.add("odom/linear.x");
  const auto pz = store.add("odom/position.z");
  nav_msgs::msg::Odometry msg;
  msg.twist.twist.linear.x = 0.7;
  msg.pose.pose.position.z = 1.1;
  publish("/test_odom", msg);
  src.read(store, 0.0);
  EXPECT_DOUBLE_EQ(store.get(vx), 0.7);
  EXPECT_DOUBLE_EQ(store.get(pz), 1.1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
