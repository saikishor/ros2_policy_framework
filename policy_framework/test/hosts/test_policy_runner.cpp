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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "policy_framework/hosts/policy_runner.hpp"
#include "policy_framework_msgs/msg/mock_fault.hpp"
#include "policy_framework_msgs/srv/switch_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using policy_framework::FaultReaction;
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

  auto client = helper->create_client<policy_framework_msgs::srv::SwitchPolicy>(
    "/runner/switch_policy");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto req = std::make_shared<policy_framework_msgs::srv::SwitchPolicy::Request>();
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

// Two StateBoundsCheck instances on entries no source provides, set directly in the state.
std::vector<rclcpp::Parameter> safety_params(
  const std::string & topic, const std::string & hot_reaction, const std::string & error_reaction)
{
  auto params = runner_params(topic);
  const std::vector<rclcpp::Parameter> more = {
    {"safety_checks", StringArray{"hot", "errors"}},
    {"hot.type", "policy_framework/StateBoundsCheck"},
    {"hot.prefixes", StringArray{"j1"}},
    {"hot.interfaces", StringArray{"temperature"}},
    {"hot.temperature.max", 80.0},
    {"errors.type", "policy_framework/StateBoundsCheck"},
    {"errors.prefixes", StringArray{"j1", "j2"}},
    {"errors.interfaces", StringArray{"error_code"}},
    {"errors.error_code.min", 0.0},
    {"errors.error_code.max", 0.0},
  };
  params.insert(params.end(), more.begin(), more.end());
  // Empty: the check keeps the default reaction.
  if (!hot_reaction.empty()) {params.push_back({"hot.on_fault", hot_reaction});}
  if (!error_reaction.empty()) {params.push_back({"errors.on_fault", error_reaction});}
  return params;
}

class RunnerSafetyTest : public PolicyRunnerTest
{
protected:
  void set(const std::string & name, double v)
  {
    runner.state().set(runner.state().index(name), v, 1.0);
  }
  double neutral()
  {
    return runner.state().get(runner.state().index(policy_framework::kNeutralCommandsEntry));
  }
};

TEST_F(RunnerSafetyTest, LoadsTheSafetyChecksAndRegistersWhatTheyRead)
{
  configure(safety_params("/runner_safety_load", "neutral_commands", "error"));
  const auto missing = runner.unprovided_state();
  for (const auto * n : {"j1/temperature", "j1/error_code", "j2/error_code"}) {
    EXPECT_NE(std::find(missing.begin(), missing.end(), n), missing.end()) << n;
  }
  const auto reactions = runner.fault_reactions();
  for (const auto r : {FaultReaction::NEUTRAL_COMMANDS, FaultReaction::ERROR}) {
    EXPECT_NE(std::find(reactions.begin(), reactions.end(), r), reactions.end());
  }
}

TEST_F(RunnerSafetyTest, NeutralCommandsLiftOnceTheCheckPassesAgain)
{
  configure(safety_params("/runner_safety_neutral", "neutral_commands", "error"));
  EXPECT_FALSE(runner.update_safety(0.0, 1.0).has_value());  // nothing received: nothing to check
  set("j1/temperature", 85.0);
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::NEUTRAL_COMMANDS);
  EXPECT_DOUBLE_EQ(neutral(), 1.0);
  set("j1/temperature", 70.0);
  EXPECT_FALSE(runner.update_safety(0.0, 1.0).has_value());
  EXPECT_DOUBLE_EQ(neutral(), 0.0);
}

TEST_F(RunnerSafetyTest, OtherReactionsAreLatchedUntilRestarted)
{
  configure(safety_params("/runner_safety_latch", "damp", "error"));
  set("j1/temperature", 85.0);
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::DAMP);
  set("j1/temperature", 70.0);
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::DAMP);
  runner.restart();
  EXPECT_FALSE(runner.update_safety(0.0, 1.0).has_value());
}

TEST_F(RunnerSafetyTest, OnlyEverEscalatesAndTheMostSevereFailingCheckWins)
{
  configure(safety_params("/runner_safety_escalate", "neutral_commands", "safe"));
  set("j1/temperature", 85.0);
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::NEUTRAL_COMMANDS);
  set("j2/error_code", 3.0);  // both fail: SAFE is more severe
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::SAFE);
  EXPECT_DOUBLE_EQ(neutral(), 0.0);  // latched: the policy is not followed any more
  set("j2/error_code", 0.0);
  set("j1/temperature", 70.0);
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::SAFE);  // never back
}

TEST_F(RunnerSafetyTest, ACheckWithoutItsOwnReactionIsAnError)
{
  configure(safety_params("/runner_safety_default", "", ""));
  set("j1/error_code", 1.0);
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::ERROR);
}

TEST_F(RunnerSafetyTest, APolicyFailureHasItsOwnReaction)
{
  configure(runner_params("/runner_safety_host_default"));
  EXPECT_EQ(runner.policy_failure("The policy step failed"), FaultReaction::ERROR);  // by default

  auto params = runner_params("/runner_safety_host");
  params.push_back({"on_policy_failure", "damp"});
  configure(params);
  EXPECT_EQ(runner.policy_failure("The policy step failed"), FaultReaction::DAMP);
  const auto reactions = runner.fault_reactions();
  EXPECT_NE(std::find(reactions.begin(), reactions.end(), FaultReaction::DAMP), reactions.end());
}

TEST_F(RunnerSafetyTest, RejectsNeutralCommandsForAPolicyFailure)
{
  // Neutral commands keep the policy running: no answer when the policy itself failed.
  auto params = runner_params("/runner_safety_host_neutral");
  params.push_back({"on_policy_failure", "neutral_commands"});
  EXPECT_ANY_THROW(configure(params));
}

TEST_F(RunnerSafetyTest, ThereIsNoGlobalOnFault)
{
  // Every check states its reaction; a global default would hide it.
  auto params = safety_params("/runner_safety_no_global", "", "");
  params.push_back({"on_fault", "hold"});  // not a parameter: ignored
  configure(params);
  set("j1/error_code", 1.0);
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::ERROR);
}

TEST_F(RunnerSafetyTest, AsksForTheReactionOfTheVerdictWhenDeclared)
{
  auto params = safety_params("/runner_safety_verdict", "neutral_commands", "neutral_commands");
  params.push_back({"errors.escalation.count", 2});
  params.push_back({"errors.escalation.on_fault", "error"});
  configure(params);
  const auto reactions = runner.fault_reactions();
  EXPECT_NE(std::find(reactions.begin(), reactions.end(), FaultReaction::ERROR), reactions.end());
  set("j1/error_code", 1.0);
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::NEUTRAL_COMMANDS);
  set("j2/error_code", 1.0);
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::ERROR);
}

TEST(PolicyRunnerInputAges, FindsTheMostSevereInputOlderThanItsMaxAge)
{
  using policy_framework::InputStamp;
  // name, stamp, from_previous_outputs, user_command, receive_time, max_age, max_age_on_fault
  const policy_framework::InputStamps stamps = {
    {"a", 9.0, false, false, 9.0, 0.5, FaultReaction::HOLD},
    {"b", 9.9, false, false, 9.9, 0.5, FaultReaction::SAFE},
    {"never", 0.0, false, false, 0.0, 0.1, FaultReaction::ERROR},  // nothing received yet
    {"unlimited", 1.0, false, false, 1.0, 0.0, FaultReaction::ERROR},  // no max_age
  };
  EXPECT_FALSE(PolicyRunner::stale_input(stamps, 9.2).has_value());
  EXPECT_EQ(PolicyRunner::stale_input(stamps, 10.0), 0U);  // a only
  EXPECT_EQ(PolicyRunner::stale_input(stamps, 10.6), 1U);  // a and b: b is more severe
}

TEST(PolicyRunnerReaction, AnUndeclaredReactionOfAVerdictIsAnError)
{
  using policy_framework::SafetyCheck;
  const std::vector<FaultReaction> declared{FaultReaction::DAMP};
  EXPECT_EQ(
    PolicyRunner::reaction_of({false, std::nullopt}, FaultReaction::HOLD, declared),
    FaultReaction::HOLD);
  EXPECT_EQ(
    PolicyRunner::reaction_of({false, FaultReaction::DAMP}, FaultReaction::HOLD, declared),
    FaultReaction::DAMP);
  EXPECT_EQ(
    PolicyRunner::reaction_of({false, FaultReaction::SAFE}, FaultReaction::HOLD, declared),
    FaultReaction::ERROR);
}

TEST_F(RunnerSafetyTest, RunsTheMonitorWithItsOwnReactions)
{
  auto params = runner_params("/runner_safety_monitor");
  params.push_back({"safety.max_input_age", 0.2});
  params.push_back({"safety.input_age_on_fault", "safe"});
  configure(params);
  EXPECT_FALSE(runner.update_safety(9.9, 10.0).has_value());
  EXPECT_EQ(runner.update_safety(9.7, 10.0), FaultReaction::SAFE);
}

TEST_F(RunnerSafetyTest, FaultsWhenASourceStopsReceivingForLongerThanItsMaxAge)
{
  auto params = runner_params("/runner_safety_max_age");
  params.push_back({"js.max_age", 0.2});
  params.push_back({"js.max_age_on_fault", "safe"});
  configure(params);
  const auto reactions = runner.fault_reactions();
  EXPECT_NE(std::find(reactions.begin(), reactions.end(), FaultReaction::SAFE), reactions.end());
  const double now = node->now().seconds();
  EXPECT_FALSE(runner.update_safety(0.0, now + 10.0).has_value());  // nothing received yet

  auto pub = helper->create_publisher<sensor_msgs::msg::JointState>("/runner_safety_max_age", 10);
  sensor_msgs::msg::JointState js;
  js.name = {"j1", "j2", "j3"};
  js.position = {1.0, 2.0, 3.0};
  for (int i = 0; i < 100 && pub->get_subscription_count() == 0; ++i) {
    spin_for(std::chrono::milliseconds(10));
  }
  pub->publish(js);
  spin_for(std::chrono::milliseconds(100));
  const double received = node->now().seconds();
  EXPECT_FALSE(runner.update_safety(0.0, received).has_value());
  EXPECT_EQ(runner.update_safety(0.0, received + 0.5), FaultReaction::SAFE);
}

// walk runs every step; jump (max_steps 1) runs one step after a switch, then walk again.
class PerPolicySafetyTest : public RunnerSafetyTest
{
protected:
  void configure_with(std::vector<rclcpp::Parameter> extra, const StringArray & jump_disables)
  {
    auto params = runner_params("/runner_per_policy");
    params.push_back({"jump.disabled_safety_checks", jump_disables});
    params.insert(params.end(), extra.begin(), extra.end());
    configure(params);
    for (const auto * n : {"j1/position", "j2/position", "j3/position"}) {set(n, 0.0);}
  }
  void step_into(const std::string & policy)
  {
    std::string msg;
    ASSERT_TRUE(runner.pipeline().request_switch(policy, msg)) << msg;
    ASSERT_EQ(runner.step(), PolicyPipeline::Status::OK);
    ASSERT_EQ(runner.pipeline().active_policy(), policy);
  }
};

TEST_F(PerPolicySafetyTest, ACheckDisabledByTheActivePolicyIsSkipped)
{
  configure_with(
    {{"safety_checks", StringArray{"hot"}}, {"hot.type", "policy_framework/StateBoundsCheck"},
      {"hot.prefixes", StringArray{"j1"}}, {"hot.interfaces", StringArray{"temperature"}},
      {"hot.temperature.max", 80.0}, {"hot.on_fault", "neutral_commands"}}, {"hot"});
  set("j1/temperature", 85.0);
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::NEUTRAL_COMMANDS);  // walk
  step_into("jump");
  EXPECT_FALSE(runner.update_safety(0.0, 1.0).has_value());
  ASSERT_EQ(runner.step(), PolicyPipeline::Status::OK);  // back to walk
  ASSERT_EQ(runner.pipeline().active_policy(), "walk");
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::NEUTRAL_COMMANDS);
}

TEST_F(PerPolicySafetyTest, AReenabledCheckStartsAfresh)
{
  // Without a reset, the JumpCheck would compare with its value from before the jump.
  configure_with(
    {{"safety_checks", StringArray{"glitch"}}, {"glitch.type", "policy_framework/JumpCheck"},
      {"glitch.prefixes", StringArray{"j1"}}, {"glitch.interfaces", StringArray{"position"}},
      {"glitch.position.max_delta", 0.5}, {"glitch.on_fault", "hold"}}, {"glitch"});
  EXPECT_FALSE(runner.update_safety(0.0, 1.0).has_value());  // walk, j1 at 0
  step_into("jump");
  set("j1/position", 3.0);
  EXPECT_FALSE(runner.update_safety(0.0, 1.0).has_value());
  ASSERT_EQ(runner.step(), PolicyPipeline::Status::OK);  // back to walk
  EXPECT_FALSE(runner.update_safety(0.0, 1.0).has_value());
  set("j1/position", 4.0);  // a real jump is still caught
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::HOLD);
}

TEST_F(PerPolicySafetyTest, AMonitorCheckCanBeDisabledAndALatchedFaultSurvivesASwitch)
{
  configure_with(
    {{"safety.max_tilt", 0.5}, {"safety.imu", "imu"}, {"safety.tilt_on_fault", "hold"}},
    {"tilt"});
  step_into("jump");
  set("imu/orientation.x", 1.0);  // upside down
  set("imu/orientation.y", 0.0);
  set("imu/orientation.z", 0.0);
  set("imu/orientation.w", 0.0);
  EXPECT_FALSE(runner.update_safety(0.0, 1.0).has_value());
  ASSERT_EQ(runner.step(), PolicyPipeline::Status::OK);  // back to walk
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::HOLD);
  step_into("jump");
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::HOLD);  // latched
}

TEST_F(PerPolicySafetyTest, RejectsAnUnknownCheckName)
{
  EXPECT_ANY_THROW(configure_with({}, {"nope"}));
}

TEST_F(PerPolicySafetyTest, RejectsAPluginCheckNamedLikeAMonitorCheck)
{
  EXPECT_ANY_THROW(
    configure_with(
      {{"safety_checks", StringArray{"tilt"}}, {"tilt.type", "policy_framework/JumpCheck"},
        {"tilt.prefixes", StringArray{"j1"}}, {"tilt.interfaces", StringArray{"position"}},
        {"tilt.position.max_delta", 0.5}}, {}));
}

TEST_F(RunnerSafetyTest, AMockSafetyCheckInjectsAFaultOnRequest)
{
  auto params = runner_params("/runner_mock_js");
  params.push_back({"safety_checks", StringArray{"mock"}});
  params.push_back({"mock.type", "policy_framework/MockSafetyCheck"});
  params.push_back({"mock.topic", "/runner_mock_fault"});
  params.push_back({"mock.on_fault", "hold"});
  params.push_back({"mock.reactions", StringArray{"damp"}});
  configure(params);
  const auto reactions = runner.fault_reactions();
  EXPECT_NE(std::find(reactions.begin(), reactions.end(), FaultReaction::DAMP), reactions.end());
  EXPECT_FALSE(runner.update_safety(0.0, 1.0).has_value());

  auto pub = helper->create_publisher<policy_framework_msgs::msg::MockFault>(
    "/runner_mock_fault", 10);
  for (int i = 0; i < 100 && pub->get_subscription_count() == 0; ++i) {
    spin_for(std::chrono::milliseconds(10));
  }
  policy_framework_msgs::msg::MockFault msg;
  msg.fault = true;
  pub->publish(msg);
  spin_for(std::chrono::milliseconds(100));
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::HOLD);  // its on_fault
  msg.reaction = "damp";
  pub->publish(msg);
  spin_for(std::chrono::milliseconds(100));
  EXPECT_EQ(runner.update_safety(0.0, 1.0), FaultReaction::DAMP);  // escalates
}

TEST_F(RunnerSafetyTest, ASourceWithoutMaxAgeIsNotChecked)
{
  configure(runner_params("/runner_safety_no_max_age"));
  auto pub = helper->create_publisher<sensor_msgs::msg::JointState>(
    "/runner_safety_no_max_age", 10);
  sensor_msgs::msg::JointState js;
  js.name = {"j1"};
  js.position = {1.0};
  for (int i = 0; i < 100 && pub->get_subscription_count() == 0; ++i) {
    spin_for(std::chrono::milliseconds(10));
  }
  pub->publish(js);
  spin_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(runner.update_safety(0.0, node->now().seconds() + 100.0).has_value());
}

TEST_F(RunnerSafetyTest, JointPositionLimitsNeedARobotDescription)
{
  auto params = runner_params("/runner_safety_limits");
  params.push_back({"safety.joint_position_limits", true});
  EXPECT_ANY_THROW(configure(params));
}

TEST_F(RunnerSafetyTest, RejectsAnUnknownReactionName)
{
  EXPECT_ANY_THROW(configure(safety_params("/runner_safety_unknown", "explode", "error")));
}

TEST_F(PolicyRunnerTest, SetsNeutralCommandsInTheStateUntilRestarted)
{
  configure(runner_params("/runner_js_neutral"));
  const auto i = runner.state().find(policy_framework::kNeutralCommandsEntry);
  ASSERT_TRUE(i.has_value());  // registered for the plugins, but provided by the runner itself
  EXPECT_TRUE(runner.unprovided_state().empty());
  runner.set_neutral_commands(true, 1.0);
  EXPECT_DOUBLE_EQ(runner.state().get(*i), 1.0);
  runner.set_neutral_commands(false, 2.0);
  EXPECT_DOUBLE_EQ(runner.state().get(*i), 0.0);
  runner.set_neutral_commands(true, 3.0);
  runner.restart();
  EXPECT_DOUBLE_EQ(runner.state().get(*i), 0.0);
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
