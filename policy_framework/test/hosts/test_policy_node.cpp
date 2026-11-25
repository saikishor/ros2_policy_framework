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
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/msg/multi_dof_command.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "policy_framework/hosts/policy_node.hpp"
#include "policy_framework_msgs/srv/switch_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

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

/// A node whose safety check fails while |j1/velocity| > 10, with the reaction @p on_fault.
std::shared_ptr<policy_framework::PolicyNode> make_checked_node(
  const std::string & on_fault, const std::vector<rclcpp::Parameter> & extra = {})
{
  auto params = node_params({});
  params.insert(params.end(), extra.begin(), extra.end());
  const std::vector<rclcpp::Parameter> more = {
    {"safety_checks", StringArray{"speed"}},
    {"speed.type", "policy_framework/StateBoundsCheck"},
    {"speed.prefixes", StringArray{"j1"}},
    {"speed.interfaces", StringArray{"velocity"}},
    {"speed.velocity.min", -10.0},
    {"speed.velocity.max", 10.0},
    {"speed.on_fault", on_fault},
  };
  params.insert(params.end(), more.begin(), more.end());
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  return std::make_shared<policy_framework::PolicyNode>(opts);
}

/// Publishes joint states with j1 at @p j1_velocity for @p ms and counts the commands received.
class CommandCounter
{
public:
  explicit CommandCounter(const std::shared_ptr<policy_framework::PolicyNode> & node)
  : helper_(std::make_shared<rclcpp::Node>("node_safety_helper"))
  {
    sub_ = helper_->create_subscription<control_msgs::msg::MultiDOFCommand>(
      "/node_reference", 10, [this](control_msgs::msg::MultiDOFCommand::SharedPtr) {++count_;});
    pub_ = helper_->create_publisher<sensor_msgs::msg::JointState>("/node_js", 10);
    exec_.add_node(node->get_node_base_interface());
    exec_.add_node(helper_);
  }

  int run(double j1_velocity, int ms)
  {
    count_ = 0;
    sensor_msgs::msg::JointState js;
    js.name = {"j1", "j2", "j3"};
    js.position = {1.0, 2.0, 3.0};
    js.velocity = {j1_velocity, 0.0, 0.0};
    for (int i = 0; i < ms / 10; ++i) {
      pub_->publish(js);
      exec_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return count_;
  }

  /// Call ~/reset_safety, as an operator would.
  void reset_safety()
  {
    auto client = helper_->create_client<std_srvs::srv::Trigger>("/policy_node/reset_safety");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(exec_.spin_until_future_complete(future, std::chrono::seconds(2)),
      rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success) << future.get()->message;
  }

private:
  rclcpp::Node::SharedPtr helper_;
  rclcpp::Subscription<control_msgs::msg::MultiDOFCommand>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  int count_ = 0;
};

}  // namespace

TEST(PolicyNode, ConfigureFailsWhenNoSourceProvidesAnInput)
{
  auto node = make_node({"j1", "j2"});  // j3 is missing and a node has no state interfaces
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(PolicyNode, ConfigureFailsWithANonPositiveUpdateRate)
{
  auto params = node_params({"j1", "j2", "j3"});
  params.push_back({"update_rate", 0.0});
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  auto node = std::make_shared<policy_framework::PolicyNode>(opts);
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(PolicyNode, KinematicsUsesTheRobotDescriptionParameter)
{
  std::ifstream f(TEST_URDF);
  std::stringstream urdf;
  urdf << f.rdbuf();
  auto params = node_params({"j1", "j2", "j3"});
  params.push_back({"robot_description", urdf.str()});
  params.push_back({"kinematics.enabled", true});
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  auto node = std::make_shared<policy_framework::PolicyNode>(opts);
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST(PolicyNode, IsDestroyedWhenReleasedWhileActive)
{
  // A reference cycle would keep the node, its publishers and DDS entities alive until process
  // exit, where unloading the middleware with its threads still running crashed.
  auto node = make_node({"j1", "j2", "j3"});
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  std::weak_ptr<policy_framework::PolicyNode> weak = node;
  node.reset();
  EXPECT_TRUE(weak.expired());
}

TEST(PolicyNode, StopsSendingCommandsAfterALatchedFaultUntilReactivated)
{
  auto node = make_checked_node("error");
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  CommandCounter counter(node);
  EXPECT_GT(counter.run(0.0, 500), 0);
  counter.run(20.0, 200);  // the fault, and the commands still in flight
  EXPECT_EQ(counter.run(20.0, 300), 0);
  EXPECT_EQ(counter.run(0.0, 300), 0);  // latched: not back by itself

  ASSERT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_GT(counter.run(0.0, 500), 0);
  node->deactivate();
}

TEST(PolicyNode, SendsCommandsAgainAfterAResetOnceTheCheckPasses)
{
  // With max_input_age: the observation of the last step before the fault is old by now, which
  // must not fault again after the reset.
  auto node = make_checked_node("error", {{"safety.max_input_age", 0.2}});
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  CommandCounter counter(node);
  EXPECT_GT(counter.run(0.0, 500), 0);
  counter.run(20.0, 200);  // the fault, and the commands still in flight
  EXPECT_EQ(counter.run(20.0, 300), 0);
  counter.reset_safety();
  EXPECT_EQ(counter.run(20.0, 300), 0);  // still failing: latched again at once
  counter.reset_safety();
  EXPECT_GT(counter.run(0.0, 500), 0);
  node->deactivate();
}

TEST(PolicyNode, KeepsSendingCommandsWithNeutralCommands)
{
  auto node = make_checked_node("neutral_commands");
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  CommandCounter counter(node);
  EXPECT_GT(counter.run(20.0, 500), 0);  // the policy keeps running
  node->deactivate();
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

  auto client = helper->create_client<policy_framework_msgs::srv::SwitchPolicy>(
    "/policy_node/switch_policy");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto req = std::make_shared<policy_framework_msgs::srv::SwitchPolicy::Request>();
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
