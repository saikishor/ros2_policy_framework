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

// End to end: several mjlab policies read their built-in velocity `command` term from one
// shared TwistSource, so the velocity command topic has a single subscription.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "policy_framework/hosts/policy_runner.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::PolicyPipeline;
using policy_framework::PolicyRunner;
using StringArray = std::vector<std::string>;

namespace
{

constexpr char kTopic[] = "/shared_cmd_vel";

void spin_for(rclcpp::Executor & exec, int ms)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
  while (std::chrono::steady_clock::now() < end) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}  // namespace

TEST(SharedCommandSource, PoliciesShareOneVelocityCommandSubscription)
{
  std::vector<rclcpp::Parameter> params = {
    {"sources", StringArray{"cmd_vel"}},
    {"cmd_vel.type", "policy_framework/TwistSource"},
    {"cmd_vel.topic", kTopic},
    {"policies", StringArray{"walk", "run"}},
  };
  for (const std::string p : {"walk", "run"}) {
    params.insert(params.end(), {
      {p + ".preprocessor", "policy_framework_mjlab/MjlabPreprocessor"},
      {p + ".model_path", std::string(TEST_MODEL)},
      {p + ".postprocessor", "policy_framework/SplitPostprocessor"},
      {p + ".output_entries", StringArray{"a/position"}},
    });
  }
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("shared_command", opts);
  auto helper = std::make_shared<rclcpp::Node>("shared_command_helper");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);
  PolicyRunner runner;
  runner.configure(node, false);

  auto pub = helper->create_publisher<geometry_msgs::msg::Twist>(kTopic, 10);
  for (int i = 0; i < 100 && pub->get_subscription_count() == 0; ++i) {spin_for(exec, 10);}
  spin_for(exec, 100);  // let discovery settle so a second subscription would show up
  EXPECT_EQ(pub->get_subscription_count(), 1u);

  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.4;
  pub->publish(msg);
  spin_for(exec, 100);
  const auto a = runner.commands().index("a/position");

  runner.read_sources(node->now().seconds());
  ASSERT_EQ(runner.step(), PolicyPipeline::Status::OK);
  EXPECT_DOUBLE_EQ(runner.commands().get(a), 0.4F);  // walk

  std::string message;
  ASSERT_TRUE(runner.pipeline().request_switch("run", message)) << message;
  runner.read_sources(node->now().seconds());
  ASSERT_EQ(runner.step(), PolicyPipeline::Status::OK);
  EXPECT_DOUBLE_EQ(runner.commands().get(a), 0.4F);  // run, from the same subscription
  EXPECT_EQ(runner.pipeline().active_policy(), "run");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
