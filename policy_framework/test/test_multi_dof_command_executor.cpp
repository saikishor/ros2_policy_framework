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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "policy_framework/multi_dof_command_executor.hpp"
#include "rclcpp/rclcpp.hpp"

using control_msgs::msg::MultiDOFCommand;
using policy_framework::DataStore;

TEST(MultiDofCommandExecutor, PublishesPositionsAsValuesAndVelocitiesAsValuesDot)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"exec.topic", "/test_reference"}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_exec", opts);
  auto sub_node = std::make_shared<rclcpp::Node>("test_sub");
  MultiDOFCommand::SharedPtr received;
  auto sub = sub_node->create_subscription<MultiDOFCommand>(
    "/test_reference", 10, [&](MultiDOFCommand::SharedPtr m) {received = m;});

  DataStore commands;
  const auto p1 = commands.add("j1/position");
  const auto v1 = commands.add("j1/velocity");
  const auto p2 = commands.add("j2/position");
  commands.add("j2/velocity");
  commands.add("j3/effort");  // neither values nor values_dot: ignored

  policy_framework::MultiDofCommandExecutor exec;
  exec.configure({node, "exec"}, commands);
  commands.set(p1, 1.0, 0.0);
  commands.set(v1, 0.1, 0.0);
  commands.set(p2, 2.0, 0.0);
  // j2/velocity never written: published as 0.

  for (int i = 0; i < 200 && !received; ++i) {
    exec.write(commands);
    rclcpp::spin_some(sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(received);
  EXPECT_EQ(received->dof_names, (std::vector<std::string>{"j1", "j2"}));
  EXPECT_EQ(received->values, (std::vector<double>{1.0, 2.0}));
  EXPECT_EQ(received->values_dot, (std::vector<double>{0.1, 0.0}));
}

TEST(MultiDofCommandExecutor, RejectsAnEmptyValuesInterface)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"exec.values_interface", ""}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_multidof_empty", opts);
  DataStore commands;
  commands.add("j1/position");
  policy_framework::MultiDofCommandExecutor exec;
  EXPECT_ANY_THROW(exec.configure({node, "exec"}, commands));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
