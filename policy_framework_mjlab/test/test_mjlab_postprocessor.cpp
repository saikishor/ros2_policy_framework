// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "policy_framework_mjlab/mjlab_postprocessor.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::DataStore;
using policy_framework::ModelInfo;
using policy_framework::PluginContext;
using policy_framework_mjlab::MjlabPostprocessor;
using StringArray = std::vector<std::string>;

namespace
{

// 3 joints, the middle one passive (not actuated by the policy).
ModelInfo model()
{
  ModelInfo m;
  m.metadata = {
    {"joint_names", "a,passive,c"},
    {"default_joint_pos", "0.1,0.2,0.3"},
    {"action_scale", "2.0,3.0"},
    {"joint_stiffness", "10.0,30.0"},
    {"joint_damping", "1.0,3.0"},
  };
  m.outputs["actions"] = {{1, 2}, {0.0F, 0.0F}};
  return m;
}

PluginContext make_context(std::vector<rclcpp::Parameter> params)
{
  params.push_back({"p.excluded_joints", StringArray{"passive"}});
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  return {std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mjlab_post", opts), "p"};
}

}  // namespace

TEST(MjlabPostprocessor, ScalesAndOffsetsActionsOfActuatedJoints)
{
  const auto m = model();
  DataStore commands;
  MjlabPostprocessor post;
  post.configure(make_context({}), m, commands);
  EXPECT_EQ(
    commands.names(),
    (StringArray{"a/position", "a/velocity", "c/position", "c/velocity"}));

  auto out = m.outputs;
  out["actions"].data = {1.0F, -1.0F};
  ASSERT_TRUE(post.compute(out, 7.5, commands));
  EXPECT_NEAR(commands.get(commands.index("a/position")), 2.0 * 1.0 + 0.1, 1e-6);
  // Commands carry the stamp of the observation they were computed from.
  EXPECT_DOUBLE_EQ(commands.stamp(commands.index("a/position")), 7.5);
  EXPECT_DOUBLE_EQ(commands.stamp(commands.index("c/velocity")), 7.5);
  EXPECT_NEAR(commands.get(commands.index("c/position")), 3.0 * -1.0 + 0.3, 1e-6);
  EXPECT_TRUE(commands.received(commands.index("c/velocity")));
  EXPECT_DOUBLE_EQ(commands.get(commands.index("c/velocity")), 0.0);
}

TEST(MjlabPostprocessor, VelocityReferenceCanBeDisabled)
{
  DataStore commands;
  MjlabPostprocessor post;
  post.configure(make_context({{"p.velocity_interface", ""}}), model(), commands);
  EXPECT_EQ(commands.names(), (StringArray{"a/position", "c/position"}));
}

TEST(MjlabPostprocessor, RejectsActionCountMismatch)
{
  rclcpp::NodeOptions opts;  // no excluded joints: 3 joints for 2 actions
  const PluginContext ctx{
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mjlab_post_bad", opts), "p"};
  DataStore commands;
  MjlabPostprocessor post;
  EXPECT_THROW(post.configure(ctx, model(), commands), std::runtime_error);
}

TEST(MakeGainParameters, NamesPidGainsPerJoint)
{
  const auto params = policy_framework_mjlab::make_gain_parameters(
    {"a", "c"}, {10.0, 30.0}, {1.0, 3.0});
  ASSERT_EQ(params.size(), 4u);
  EXPECT_EQ(params[0].get_name(), "gains.a.p");
  EXPECT_DOUBLE_EQ(params[0].as_double(), 10.0);
  EXPECT_EQ(params[3].get_name(), "gains.c.d");
  EXPECT_DOUBLE_EQ(params[3].as_double(), 3.0);
  EXPECT_THROW(
    policy_framework_mjlab::make_gain_parameters({"a", "c"}, {10.0}, {1.0, 3.0}),
    std::invalid_argument);
}

TEST(MjlabPostprocessor, SendsPolicyGainsToTheControllerOnReset)
{
  auto pid = std::make_shared<rclcpp::Node>("fake_pid");
  for (const auto * n : {"gains.a.p", "gains.a.d", "gains.c.p", "gains.c.d"}) {
    pid->declare_parameter<double>(n, 0.0);
  }
  DataStore commands;
  MjlabPostprocessor post;
  const auto ctx = make_context({{"p.gains_controller", "/fake_pid"}});
  post.configure(ctx, model(), commands);
  post.reset();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(pid);
  exec.add_node(ctx.node->get_node_base_interface());
  for (int i = 0; i < 300 && pid->get_parameter("gains.c.d").as_double() < 1.0; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_DOUBLE_EQ(pid->get_parameter("gains.a.p").as_double(), 10.0);
  EXPECT_DOUBLE_EQ(pid->get_parameter("gains.c.d").as_double(), 3.0);
}

TEST(MjlabPostprocessor, OnlyTheLatestActivatedPolicyGainsAreSent)
{
  auto pid = std::make_shared<rclcpp::Node>("fake_pid_latest");
  for (const auto * n : {"gains.a.p", "gains.a.d", "gains.c.p", "gains.c.d"}) {
    pid->declare_parameter<double>(n, 0.0);
  }
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    {"walk.excluded_joints", StringArray{"passive"}},
    {"walk.gains_controller", "/fake_pid_latest"},
    {"jump.excluded_joints", StringArray{"passive"}},
    {"jump.gains_controller", "/fake_pid_latest"}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_latest_gains", opts);
  auto jump_model = model();
  jump_model.metadata["joint_stiffness"] = "99.0,99.0";
  DataStore commands;
  MjlabPostprocessor walk, jump;
  walk.configure({node, "walk"}, model(), commands);
  jump.configure({node, "jump"}, jump_model, commands);

  // Two activations within one gains period: jump, then walk. walk was configured first, so
  // its timer fires first: without latest-wins, jump's gains would land last.
  jump.reset();
  walk.reset();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(pid);
  exec.add_node(node->get_node_base_interface());
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(2500);
  while (std::chrono::steady_clock::now() < end) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_DOUBLE_EQ(pid->get_parameter("gains.a.p").as_double(), 10.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
