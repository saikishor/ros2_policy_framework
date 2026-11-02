// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <thread>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "policy_framework/kinematics.hpp"
#include "policy_framework/policy_controller.hpp"
#include "rclcpp/rclcpp.hpp"

using controller_interface::interface_configuration_type;
using hardware_interface::CommandInterface;
using hardware_interface::InterfaceDescription;
using hardware_interface::InterfaceInfo;
using hardware_interface::StateInterface;
using lifecycle_msgs::msg::State;
using StringArray = std::vector<std::string>;

namespace
{

InterfaceDescription describe(const std::string & full_name)
{
  const auto slash = full_name.rfind('/');
  InterfaceInfo info;
  info.name = full_name.substr(slash + 1);
  info.data_type = "double";
  info.initial_value = "0.0";
  return InterfaceDescription(full_name.substr(0, slash), info);
}

class PolicyControllerTest : public ::testing::Test
{
protected:
  void init(
    std::vector<rclcpp::Parameter> extra = {}, unsigned int update_rate = 50,
    const std::string & robot_description = "")
  {
    std::vector<rclcpp::Parameter> params = {
      {"policies", StringArray{"walk"}},
      {"walk.preprocessor", "policy_framework/ConcatPreprocessor"},
      {"walk.input_entries", StringArray{"j1/position", "j2/position", "j3/position"}},
      {"walk.model_path", std::string(TEST_MODELS_DIR) + "/affine.onnx"},
      {"walk.postprocessor", "policy_framework/SplitPostprocessor"},
      {"walk.output_entries", StringArray{"j1/position", "j2/position"}},
    };
    if (with_reference_executor) {
      params.push_back({"executors", StringArray{"ref"}});
      params.push_back({"ref.type", "policy_framework/ReferenceInterfaceExecutor"});
      params.push_back({"ref.prefix", "pid"});
    }
    params.insert(params.end(), extra.begin(), extra.end());
    controller_interface::ControllerInterfaceParams p;
    p.controller_name = "policy_controller";
    p.update_rate = update_rate;
    p.controller_manager_update_rate = update_rate;
    p.node_options.parameter_overrides(params);
    p.robot_description = robot_description;
    ASSERT_EQ(controller.init(p), controller_interface::return_type::OK);
  }

  /// Create interfaces for the controller's requested names and loan them to it.
  void assign()
  {
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    for (const auto & n : controller.state_interface_configuration().names) {
      states.push_back(std::make_shared<StateInterface>(describe(n)));
      loaned_states.emplace_back(states.back());
    }
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    for (const auto & n : controller.command_interface_configuration().names) {
      commands.push_back(std::make_shared<CommandInterface>(describe(n)));
      loaned_commands.emplace_back(commands.back(), nullptr);
    }
    controller.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
  }

  void set_state(const std::string & name, double v)
  {
    for (auto & s : states) {
      if (s->get_name() == name) {ASSERT_TRUE(s->set_value(v));}
    }
  }

  double command(const std::string & name)
  {
    for (auto & c : commands) {
      if (c->get_name() == name) {return c->get_optional().value();}
    }
    ADD_FAILURE() << "no command interface " << name;
    return 0.0;
  }

  controller_interface::return_type update()
  {
    return controller.update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 20000000));
  }

  bool with_reference_executor = true;
  policy_framework::PolicyController controller;
  std::vector<StateInterface::SharedPtr> states;
  std::vector<CommandInterface::SharedPtr> commands;
};

}  // namespace

TEST_F(PolicyControllerTest, ClaimsUnprovidedInputsAsStateInterfacesAndCommandsWithPrefix)
{
  init();
  ASSERT_EQ(controller.configure().id(), State::PRIMARY_STATE_INACTIVE);
  const auto state_cfg = controller.state_interface_configuration();
  EXPECT_EQ(state_cfg.type, interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(state_cfg.names, (StringArray{"j1/position", "j2/position", "j3/position"}));
  const auto cmd_cfg = controller.command_interface_configuration();
  EXPECT_EQ(cmd_cfg.names, (StringArray{"pid/j1/position", "pid/j2/position"}));
}

TEST_F(PolicyControllerTest, ClaimsTheKinematicsInputsFromTheRobotDescription)
{
  std::ifstream f(TEST_URDF);
  std::stringstream urdf;
  urdf << f.rdbuf();
  init({{"kinematics.enabled", true}}, 50, urdf.str());
  ASSERT_EQ(controller.configure().id(), State::PRIMARY_STATE_INACTIVE);
  auto names = controller.state_interface_configuration().names;
  std::sort(names.begin(), names.end());
  EXPECT_EQ(
    names, (StringArray{"j1/position", "j1/velocity", "j2/position", "j2/velocity",
      "j3/position", "j3/velocity"}));
}

TEST_F(PolicyControllerTest, KinematicsIsUpdatedOnEveryUpdateNotOnlyPolicySteps)
{
  std::ifstream f(TEST_URDF);
  std::stringstream urdf;
  urdf << f.rdbuf();
  init({{"kinematics.enabled", true}, {"policy_rate", 50.0}}, 100, urdf.str());
  ASSERT_EQ(controller.configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign();
  ASSERT_EQ(controller.get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  const auto kin = controller.kinematics();
  ASSERT_NE(kin, nullptr);
  const auto link1 = kin->frame("link1");
  ASSERT_EQ(update(), controller_interface::return_type::OK);  // policy step
  ASSERT_TRUE(kin->ready());
  EXPECT_TRUE(kin->transform(link1).linear().isIdentity(1e-12));

  set_state("j1/position", M_PI / 2);
  ASSERT_EQ(update(), controller_interface::return_type::OK);  // no policy step
  EXPECT_TRUE(
    kin->transform(link1).linear().isApprox(
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix()));
}

TEST_F(PolicyControllerTest, InputsProvidedByTopicSourcesAreNotClaimed)
{
  init({
    {"sources", StringArray{"js"}},
    {"js.type", "policy_framework/JointStateSource"},
    {"js.joints", StringArray{"j3"}}});
  ASSERT_EQ(controller.configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    controller.state_interface_configuration().names,
    (StringArray{"j1/position", "j2/position"}));
}

TEST_F(PolicyControllerTest, UpdateRunsPolicyFromStateToCommandInterfaces)
{
  init();
  ASSERT_EQ(controller.configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign();
  ASSERT_EQ(controller.get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  set_state("j1/position", 1.0);
  set_state("j2/position", 2.0);
  set_state("j3/position", 3.0);

  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(command("pid/j1/position"), 4.5);
  EXPECT_DOUBLE_EQ(command("pid/j2/position"), 6.5);
}

TEST_F(PolicyControllerTest, WithoutExecutorsNoCommandInterfaceIsClaimed)
{
  with_reference_executor = false;
  init();
  ASSERT_EQ(controller.configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    controller.command_interface_configuration().type, interface_configuration_type::NONE);
}

TEST_F(PolicyControllerTest, PdEffortExecutorComputesEffortAndZeroesItOnDeactivation)
{
  with_reference_executor = false;
  init({
    {"executors", StringArray{"pd"}},
    {"pd.type", "policy_framework/PdEffortExecutor"},
    {"pd.stiffness", std::vector<double>{10.0, 10.0}},
    {"pd.damping", std::vector<double>{1.0, 1.0}}});
  ASSERT_EQ(controller.configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    controller.state_interface_configuration().names,
    (StringArray{"j1/position", "j2/position", "j3/position", "j1/velocity", "j2/velocity"}));
  EXPECT_EQ(
    controller.command_interface_configuration().names, (StringArray{"j1/effort", "j2/effort"}));
  assign();
  ASSERT_EQ(controller.get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  set_state("j1/position", 1.0);
  set_state("j2/position", 2.0);
  set_state("j3/position", 3.0);
  set_state("j1/velocity", 0.5);

  ASSERT_EQ(update(), controller_interface::return_type::OK);
  // References 4.5 and 6.5 (see UpdateRunsPolicyFromStateToCommandInterfaces).
  EXPECT_DOUBLE_EQ(command("j1/effort"), 10.0 * (4.5 - 1.0) + 1.0 * (0.0 - 0.5));
  EXPECT_DOUBLE_EQ(command("j2/effort"), 10.0 * (6.5 - 2.0));

  ASSERT_EQ(controller.get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_DOUBLE_EQ(command("j1/effort"), 0.0);
  EXPECT_DOUBLE_EQ(command("j2/effort"), 0.0);
}

TEST_F(PolicyControllerTest, PolicyRunsAtPolicyRateAndExecutorsOnEveryUpdate)
{
  with_reference_executor = false;
  init({
    {"policy_rate", 50.0},
    {"executors", StringArray{"pd"}},
    {"pd.type", "policy_framework/PdEffortExecutor"},
    {"pd.stiffness", std::vector<double>{10.0, 10.0}},
    {"pd.damping", std::vector<double>{0.0, 0.0}}}, 100);
  ASSERT_EQ(controller.configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign();
  ASSERT_EQ(controller.get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  set_state("j1/position", 1.0);
  set_state("j2/position", 2.0);
  set_state("j3/position", 3.0);
  ASSERT_EQ(update(), controller_interface::return_type::OK);  // policy step: j1 ref 4.5
  EXPECT_DOUBLE_EQ(command("j1/effort"), 10.0 * (4.5 - 1.0));

  set_state("j1/position", 2.0);  // a policy step would move the j1 reference
  ASSERT_EQ(update(), controller_interface::return_type::OK);  // no policy step, PD only
  EXPECT_DOUBLE_EQ(command("j1/effort"), 10.0 * (4.5 - 2.0));

  ASSERT_EQ(update(), controller_interface::return_type::OK);  // policy step again
  EXPECT_NE(command("j1/effort"), 10.0 * (4.5 - 2.0));
}

TEST_F(PolicyControllerTest, NegativePolicyRateIsRejected)
{
  init({{"policy_rate", -50.0}});
  EXPECT_EQ(controller.configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(PolicyControllerTest, AsyncInferenceKeepsThePdLawInTheUpdateLoop)
{
  with_reference_executor = false;
  init({
    {"policy_rate", 50.0},
    {"async_inference", true},
    {"executors", StringArray{"pd"}},
    {"pd.type", "policy_framework/PdEffortExecutor"},
    {"pd.stiffness", std::vector<double>{10.0, 10.0}},
    {"pd.damping", std::vector<double>{0.0, 0.0}}}, 100);
  ASSERT_EQ(controller.configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign();
  ASSERT_EQ(controller.get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  set_state("j1/position", 1.0);
  set_state("j2/position", 2.0);
  set_state("j3/position", 3.0);

  // Triggers the first policy step without waiting for it: nothing to send yet.
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(command("j1/effort"), 0.0);

  // Once the step finished, its references (j1 4.5) apply from the next update, PD on the
  // latest state. Polled rather than a fixed sleep: the first inference can be slow on a
  // loaded machine.
  set_state("j1/position", 2.0);
  for (int i = 0; i < 200 && command("j1/effort") <= 0.0; ++i) {  // 0 until applied
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  EXPECT_DOUBLE_EQ(command("j1/effort"), 10.0 * (4.5 - 2.0));

  ASSERT_EQ(controller.get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_DOUBLE_EQ(command("j1/effort"), 0.0);
}

TEST_F(PolicyControllerTest, AsyncInferenceAndIsAsyncAreExclusive)
{
  init({{"async_inference", true}, {"is_async", true}});
  EXPECT_EQ(controller.configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(PolicyControllerTest, PolicyRateMustDivideTheUpdateRate)
{
  init({{"policy_rate", 30.0}}, 100);
  EXPECT_EQ(controller.configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(PolicyControllerTest, TopicExecutorsAreRejected)
{
  with_reference_executor = false;
  init({
    {"executors", StringArray{"pub"}},
    {"pub.type", "policy_framework/MultiDofCommandExecutor"}});
  EXPECT_EQ(controller.configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(PolicyControllerTest, ExecutorFeedbackMustComeFromStateInterfaces)
{
  with_reference_executor = false;
  init({
    {"sources", StringArray{"js"}},
    {"js.type", "policy_framework/JointStateSource"},  // provides every joint
    {"executors", StringArray{"pd"}},
    {"pd.type", "policy_framework/PdEffortExecutor"},
    {"pd.stiffness", std::vector<double>{10.0, 10.0}},
    {"pd.damping", std::vector<double>{1.0, 1.0}}});
  EXPECT_EQ(controller.configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(PolicyControllerTest, TwoExecutorsCannotClaimTheSameInterface)
{
  init({
    {"executors", StringArray{"ref", "ref2"}},
    {"ref2.type", "policy_framework/ReferenceInterfaceExecutor"},
    {"ref2.prefix", "pid"}});
  EXPECT_EQ(controller.configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
