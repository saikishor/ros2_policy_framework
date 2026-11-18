// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "policy_framework/policy_runner.hpp"
#include "policy_framework/srv/switch_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using policy_framework::PolicyPipeline;
using policy_framework::PolicyRunner;
using StringArray = std::vector<std::string>;

namespace
{

const std::string kModel = std::string(TEST_MODELS_DIR) + "/affine.onnx";

// Two policies on the affine test model: actions = [p1 + p3 + 0.5, 2 p2 + p3 - 0.5]
std::vector<rclcpp::Parameter> runner_params(const std::string & topic)
{
  return {
    {"sources", StringArray{"js"}},
    {"js.type", "policy_framework/JointStateSource"},
    {"js.topic", topic},
    {"policies", StringArray{"walk", "jump"}},
    {"walk.preprocessor", "policy_framework/ConcatPreprocessor"},
    {"walk.input_entries", StringArray{"j1/position", "j2/position", "j3/position"}},
    {"walk.model_path", kModel},
    {"walk.postprocessor", "policy_framework/SplitPostprocessor"},
    {"walk.output_entries", StringArray{"j1/position", "j2/position"}},
    {"jump.preprocessor", "policy_framework/ConcatPreprocessor"},
    {"jump.input_entries", StringArray{"j3/position", "j2/position", "j1/position"}},
    {"jump.model_path", kModel},
    {"jump.postprocessor", "policy_framework/SplitPostprocessor"},
    {"jump.output_entries", StringArray{"j1/position", "j2/position"}},
    {"jump.max_steps", 1},
    {"jump.interruptible", false},
  };
}

class PolicyRunnerTest : public ::testing::Test
{
protected:
  void configure(
    const std::vector<rclcpp::Parameter> & params, const std::string & robot_description = "")
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(params);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("runner", opts);
    helper = std::make_shared<rclcpp::Node>("helper");
    exec.add_node(node->get_node_base_interface());
    exec.add_node(helper);
    runner.configure(node, true, robot_description, step_in_other_thread);
  }

  bool step_in_other_thread = false;

  void spin_for(std::chrono::milliseconds d)
  {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  double command(const std::string & n) {return runner.commands().get(runner.commands().index(n));}

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr helper;
  rclcpp::executors::SingleThreadedExecutor exec;
  PolicyRunner runner;
};

}  // namespace

TEST_F(PolicyRunnerTest, RunsPolicyFedByTopicSourceAndSwitchesViaService)
{
  configure(runner_params("/runner_js"));
  EXPECT_TRUE(runner.unprovided_state().empty());
  EXPECT_EQ(runner.commands().size(), 2u);

  runner.read_sources(0.0);
  EXPECT_EQ(runner.step(), PolicyPipeline::Status::WAITING_FOR_DATA);

  auto pub = helper->create_publisher<sensor_msgs::msg::JointState>("/runner_js", 10);
  sensor_msgs::msg::JointState js;
  js.name = {"j1", "j2", "j3"};
  js.position = {1.0, 2.0, 3.0};
  for (int i = 0; i < 100 && pub->get_subscription_count() == 0; ++i) {spin_for(std::chrono::milliseconds(10));}
  pub->publish(js);
  spin_for(std::chrono::milliseconds(100));

  runner.read_sources(0.0);
  ASSERT_EQ(runner.step(), PolicyPipeline::Status::OK);
  EXPECT_DOUBLE_EQ(command("j1/position"), 1.0 + 3.0 + 0.5);
  EXPECT_DOUBLE_EQ(command("j2/position"), 2 * 2.0 + 3.0 - 0.5);

  auto client = helper->create_client<policy_framework::srv::SwitchPolicy>("/runner/switch_policy");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto req = std::make_shared<policy_framework::srv::SwitchPolicy::Request>();
  req->policy_name = "jump";
  auto future = client->async_send_request(req);
  ASSERT_EQ(exec.spin_until_future_complete(future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success) << future.get()->message;

  ASSERT_EQ(runner.step(), PolicyPipeline::Status::OK);
  EXPECT_EQ(runner.pipeline().active_policy(), "jump");
  EXPECT_DOUBLE_EQ(command("j1/position"), 3.0 + 1.0 + 0.5);
  runner.step();  // max_steps reached on the previous step: back to the default
  EXPECT_EQ(runner.pipeline().active_policy(), "walk");
}

TEST_F(PolicyRunnerTest, ReportsStateEntriesNoSourceProvides)
{
  auto params = runner_params("/runner_js2");
  params.push_back({"js.joints", StringArray{"j1", "j2"}});
  configure(params);
  EXPECT_EQ(runner.unprovided_state(), StringArray{"j3/position"});
}

TEST_F(PolicyRunnerTest, UpdatesTheKinematicsOnceEveryJointWasReceived)
{
  auto params = runner_params("/runner_js3");
  params.push_back({"kinematics.enabled", true});
  params.push_back({"kinematics.joints", StringArray{"j1", "j2"}});
  std::ifstream f(TEST_URDF);
  std::stringstream urdf;
  urdf << f.rdbuf();
  configure(params, urdf.str());
  auto kin = runner.kinematics();
  ASSERT_NE(kin, nullptr);
  const auto imu = kin->frame("imu_link");

  auto & state = runner.state();
  for (const auto * j : {"j1/position", "j2/position", "j1/velocity"}) {
    state.set(state.index(j), 0.0, 2.0);
  }
  runner.update_kinematics();
  EXPECT_FALSE(kin->ready());  // j2/velocity missing
  state.set(state.index("j2/velocity"), 0.0, 3.0);
  runner.update_kinematics();
  ASSERT_TRUE(kin->ready());
  EXPECT_DOUBLE_EQ(kin->stamp(), 2.0);  // oldest input
  EXPECT_NEAR(kin->transform(imu).translation().x(), 1.0, 1e-9);
}

TEST_F(PolicyRunnerTest, PoliciesShareTheHostKinematicsWhenTheyStepInItsThread)
{
  auto params = runner_params("/runner_js5");
  params.push_back({"kinematics.enabled", true});
  std::ifstream f(TEST_URDF);
  std::stringstream urdf;
  urdf << f.rdbuf();
  configure(params, urdf.str());
  EXPECT_EQ(runner.policy_kinematics(), runner.kinematics());
}

TEST_F(PolicyRunnerTest, PolicyKinematicsFollowsTheStateOfItsStepInAnotherThread)
{
  step_in_other_thread = true;
  auto params = runner_params("/runner_js6");
  params.push_back({"kinematics.enabled", true});
  std::ifstream f(TEST_URDF);
  std::stringstream urdf;
  urdf << f.rdbuf();
  configure(params, urdf.str());
  const auto host = runner.kinematics();
  const auto policy = runner.policy_kinematics();
  ASSERT_NE(policy, nullptr);
  ASSERT_NE(policy, host);
  const auto imu = host->frame("imu_link");

  // The snapshot handed to the step: every joint at zero.
  auto & state = runner.state();
  for (const auto & n : state.names()) {state.set(state.index(n), 0.0, 1.0);}
  auto snapshot = state;
  auto commands = runner.commands();
  // Meanwhile the control loop moves on: j1 turned by 90 degrees.
  state.set(state.index("j1/position"), M_PI / 2.0, 2.0);
  runner.update_kinematics();
  EXPECT_FALSE(policy->ready());  // only updated by the step

  ASSERT_EQ(runner.step(snapshot, commands), policy_framework::PolicyPipeline::Status::OK);
  EXPECT_NEAR(policy->transform(imu).translation().x(), 1.0, 1e-9);  // the snapshot
  EXPECT_DOUBLE_EQ(policy->stamp(), 1.0);
  EXPECT_NEAR(host->transform(imu).translation().x(), 0.0, 1e-9);    // the latest state
  EXPECT_NEAR(host->transform(imu).translation().y(), 1.0, 1e-9);
}

TEST_F(PolicyRunnerTest, HasNoKinematicsByDefault)
{
  configure(runner_params("/runner_js4"));
  EXPECT_EQ(runner.kinematics(), nullptr);
}

TEST_F(PolicyRunnerTest, ChecksTheSafetyOfTheStateItReads)
{
  auto params = runner_params("/runner_js9");
  params.push_back({"safety.max_joint_velocity", 5.0});
  params.push_back({"safety.joint_position_limits", true});
  params.push_back({"safety.position_margin", 0.2});
  params.push_back({"safety.max_input_age", 0.5});
  std::ifstream f(TEST_URDF);
  std::stringstream urdf;
  urdf << f.rdbuf();
  configure(params, urdf.str());
  auto & state = runner.state();
  EXPECT_TRUE(state.find("j1/velocity").has_value());  // registered for the controller to claim
  EXPECT_FALSE(runner.check_safety(0.0, 0.0));

  state.set(state.index("j2/position"), 3.1, 0.0);  // limit 3.2 minus the margin
  const auto v = runner.check_safety(0.0, 0.0);
  ASSERT_TRUE(v);
  EXPECT_NE(v->find("joint j2"), std::string::npos) << *v;
  state.set(state.index("j2/position"), 0.0, 0.0);
  EXPECT_TRUE(runner.check_safety(1.0, 2.0));  // a one second old observation
}

TEST_F(PolicyRunnerTest, RunsTheSafetyCheckPluginsAndResetsThemOnRestart)
{
  auto params = runner_params("/runner_js10");
  params.push_back({"safety_checks", StringArray{"motors"}});
  params.push_back({"motors.type", "policy_framework/StateBoundsCheck"});
  params.push_back({"motors.prefixes", StringArray{"j1"}});
  params.push_back({"motors.interfaces", StringArray{"effort"}});
  params.push_back({"motors.effort.max", 10.0});
  params.push_back({"motors.trigger_cycles", 2});
  configure(params);
  runner.restart();
  auto & state = runner.state();
  EXPECT_FALSE(runner.check_safety(0.0, 0.0));

  state.set(state.index("j1/effort"), 20.0, 0.0);
  EXPECT_FALSE(runner.check_safety(0.0, 0.0));  // the first violation
  const auto v = runner.check_safety(0.0, 0.0);
  ASSERT_TRUE(v);
  EXPECT_EQ(v->rfind("motors: j1/effort", 0), 0u) << *v;

  runner.restart();  // a reactivation starts the count again
  EXPECT_FALSE(runner.check_safety(0.0, 0.0));
}

TEST(PolicyRunner, JointPositionLimitsNeedARobotDescription)
{
  auto params = runner_params("/unused_js");
  params.push_back({"safety.joint_position_limits", true});
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("runner_no_urdf", opts);
  PolicyRunner runner;
  EXPECT_THROW(runner.configure(node), std::runtime_error);
}

TEST(PolicyRunner, ConfigureFailsWithoutPolicies)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("empty_runner");
  PolicyRunner runner;
  EXPECT_THROW(runner.configure(node), std::runtime_error);
}

TEST(PolicyRunner, ConfigureRejectsInvalidPolicyParameters)
{
  auto params = runner_params("/unused_js");
  params.push_back({"jump.max_steps", -1});  // overrides the earlier value
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("runner_invalid", opts);
  policy_framework::PolicyRunner runner;
  EXPECT_ANY_THROW(runner.configure(node));
}

TEST(PolicyRunner, ConfigureRequiresTheStagesOfEveryPolicy)
{
  auto params = runner_params("/unused_js");
  params.erase(params.begin() + 4);  // walk.preprocessor
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("runner_missing", opts);
  policy_framework::PolicyRunner runner;
  EXPECT_ANY_THROW(runner.configure(node));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
