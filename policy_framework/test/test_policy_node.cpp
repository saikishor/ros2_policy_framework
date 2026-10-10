// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/msg/multi_dof_command.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "policy_framework/policy_node.hpp"
#include "policy_framework/srv/switch_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using lifecycle_msgs::msg::State;
using StringArray = std::vector<std::string>;

namespace
{

std::vector<rclcpp::Parameter> node_params(const StringArray & provided_joints)
{
  return {
    {"update_rate", 100.0},
    {"sources", StringArray{"js"}},
    {"js.type", "policy_framework/JointStateSource"},
    {"js.topic", "/node_js"},
    {"js.joints", provided_joints},
    {"policies", StringArray{"walk"}},
    {"walk.preprocessor", "policy_framework/ConcatPreprocessor"},
    {"walk.input_entries", StringArray{"j1/position", "j2/position", "j3/position"}},
    {"walk.model_path", std::string(TEST_MODELS_DIR) + "/affine.onnx"},
    {"walk.postprocessor", "policy_framework/SplitPostprocessor"},
    {"walk.output_entries", StringArray{"j1/position", "j2/position"}},
    {"executors", StringArray{"ref"}},
    {"ref.type", "policy_framework/MultiDofCommandExecutor"},
    {"ref.topic", "/node_reference"},
  };
}

std::shared_ptr<policy_framework::PolicyNode> make_node(const StringArray & provided_joints)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(node_params(provided_joints));
  return std::make_shared<policy_framework::PolicyNode>(opts);
}

}  // namespace

TEST(PolicyNode, ConfigureFailsWhenNoSourceProvidesAnInput)
{
  auto node = make_node({"j1", "j2"});  // j3 is missing and a node has no state interfaces
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(PolicyNode, ActiveNodePublishesCommandsComputedFromTopics)
{
  auto node = make_node({});
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  auto helper = std::make_shared<rclcpp::Node>("node_test_helper");
  control_msgs::msg::MultiDOFCommand::SharedPtr received;
  auto sub = helper->create_subscription<control_msgs::msg::MultiDOFCommand>(
    "/node_reference", 10, [&](control_msgs::msg::MultiDOFCommand::SharedPtr m) {received = m;});
  auto pub = helper->create_publisher<sensor_msgs::msg::JointState>("/node_js", 10);
  sensor_msgs::msg::JointState js;
  js.name = {"j1", "j2", "j3"};
  js.position = {1.0, 2.0, 3.0};

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);
  for (int i = 0; i < 300 && !received; ++i) {
    pub->publish(js);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(received);
  EXPECT_EQ(received->dof_names, (StringArray{"j1", "j2"}));
  EXPECT_DOUBLE_EQ(received->values[0], 4.5);
  EXPECT_DOUBLE_EQ(received->values[1], 6.5);

  EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST(PolicyNode, ReactivationRestartsTheDefaultPolicy)
{
  auto params = node_params({});
  params.push_back({"policies", StringArray{"walk", "jump"}});
  params.push_back({"jump.preprocessor", "policy_framework/ConcatPreprocessor"});
  params.push_back({"jump.input_entries", StringArray{"j3/position", "j2/position", "j1/position"}});
  params.push_back({"jump.model_path", std::string(TEST_MODELS_DIR) + "/affine.onnx"});
  params.push_back({"jump.postprocessor", "policy_framework/SplitPostprocessor"});
  params.push_back({"jump.output_entries", StringArray{"j1/position", "j2/position"}});
  params.push_back({"jump.interruptible", false});
  params.push_back({"ref.topic", "/node_reference_restart"});
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  auto node = std::make_shared<policy_framework::PolicyNode>(opts);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  auto helper = std::make_shared<rclcpp::Node>("node_restart_helper");
  double j2 = 0.0;
  auto sub = helper->create_subscription<control_msgs::msg::MultiDOFCommand>(
    "/node_reference_restart", 10,
    [&](control_msgs::msg::MultiDOFCommand::SharedPtr m) {j2 = m->values[1];});
  auto pub = helper->create_publisher<sensor_msgs::msg::JointState>("/node_js", 10);
  sensor_msgs::msg::JointState js;
  js.name = {"j1", "j2", "j3"};
  js.position = {1.0, 2.0, 3.0};
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);
  // walk: j2 = 2 * 2 + 3 - 0.5 = 6.5; jump (inputs reversed): j2 = 2 * 2 + 1 - 0.5 = 4.5
  auto wait_for_j2 = [&](double expected) {
      for (int i = 0; i < 300 && std::abs(j2 - expected) > 1e-9; ++i) {
        pub->publish(js);
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      return j2;
    };
  ASSERT_DOUBLE_EQ(wait_for_j2(6.5), 6.5);

  auto client = helper->create_client<policy_framework::srv::SwitchPolicy>(
    "/policy_node/switch_policy");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto req = std::make_shared<policy_framework::srv::SwitchPolicy::Request>();
  req->policy_name = "jump";
  auto future = client->async_send_request(req);
  ASSERT_EQ(
    exec.spin_until_future_complete(future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_DOUBLE_EQ(wait_for_j2(4.5), 4.5);

  ASSERT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_DOUBLE_EQ(wait_for_j2(6.5), 6.5);  // back to the default, although jump is not interruptible
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
