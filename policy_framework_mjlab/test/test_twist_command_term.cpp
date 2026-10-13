// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "policy_framework_mjlab/twist_command_term.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::DataStore;
using policy_framework::ModelInfo;
using policy_framework::TensorMap;
using policy_framework_mjlab::TwistCommandTerm;

TEST(TwistCommandTerm, IsLinearXYAngularZOfTheTopicAndZeroWhenAbsentOrStale)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"p.terms.command.topic", "/test_twist_term"},
      {"p.terms.command.timeout", 0.3}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_twist_term", opts);
  DataStore state;
  TwistCommandTerm term;
  ASSERT_EQ(term.configure({node, "p.terms.command"}, ModelInfo{}, state), 3U);

  // A policy must not wait for a velocity command to start: ready and zero without one.
  float out[3] = {9.0F, 9.0F, 9.0F};
  EXPECT_TRUE(term.ready(state));
  // A zero command is generated now, so it is stamped now.
  double before = node->now().seconds();
  EXPECT_GE(term.compute(state, TensorMap{}, out), before);
  EXPECT_FLOAT_EQ(out[0], 0.0F);
  EXPECT_FLOAT_EQ(out[1], 0.0F);
  EXPECT_FLOAT_EQ(out[2], 0.0F);

  auto pub_node = std::make_shared<rclcpp::Node>("test_twist_term_pub");
  auto pub = pub_node->create_publisher<geometry_msgs::msg::Twist>("/test_twist_term", 10);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.6;
  msg.linear.y = -0.1;
  msg.linear.z = 5.0;  // not part of the mjlab command
  msg.angular.z = 0.3;
  double stamp = 0.0;
  for (int i = 0; i < 500 && out[0] < 0.5F; ++i) {
    pub->publish(msg);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    stamp = term.compute(state, TensorMap{}, out);
  }
  // A received command is stamped with its receive time (a Twist has no header).
  EXPECT_GT(stamp, before);
  EXPECT_LE(stamp, node->now().seconds());
  EXPECT_FLOAT_EQ(out[0], 0.6F);
  EXPECT_FLOAT_EQ(out[1], -0.1F);
  EXPECT_FLOAT_EQ(out[2], 0.3F);

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  before = node->now().seconds();
  EXPECT_GE(term.compute(state, TensorMap{}, out), before);
  EXPECT_FLOAT_EQ(out[0], 0.0F);
  EXPECT_FLOAT_EQ(out[2], 0.0F);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
