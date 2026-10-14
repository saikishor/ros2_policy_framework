// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "policy_framework_mjlab/mjlab_preprocessor.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::DataStore;
using policy_framework::ModelInfo;
using policy_framework::PluginContext;
using policy_framework::Tensor;
using policy_framework::TensorMap;
using policy_framework_mjlab::MjlabPreprocessor;
using Floats = std::vector<float>;

namespace
{

Tensor zeros(int64_t n) {return {{1, n}, std::vector<float>(static_cast<size_t>(n), 0.0F)};}

PluginContext make_context(std::vector<rclcpp::Parameter> params = {})
{
  rclcpp::NodeOptions opts;
  params.emplace_back("p.imu_name", "imu");
  opts.parameter_overrides(params);
  return {std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mjlab_pre", opts), "p"};
}

// 2 joints, 1 action, twist command; single obs = 3 + 2 + 2 + 1 + 3 + 3 = 14.
ModelInfo twist_model(int64_t history)
{
  ModelInfo m;
  m.metadata = {
    {"joint_names", "a,b"},
    {"default_joint_pos", "0.1,0.2"},
    {"observation_names", "base_ang_vel,joint_pos,joint_vel,actions,command,imu_projected_gravity"},
    {"command_names", "twist"},
    {"action_scale", "1.0"},
  };
  m.inputs["obs"] = zeros(14 * history);
  m.outputs["actions"] = zeros(1);
  return m;
}

// command (2 * 2 joints) + joint_pos (2) = 6, with time_step input and motion outputs.
ModelInfo motion_model()
{
  ModelInfo m;
  m.metadata = {
    {"joint_names", "a,b"},
    {"default_joint_pos", "0.1,0.2"},
    {"observation_names", "command,joint_pos"},
    {"command_names", "motion"},
  };
  m.inputs["obs"] = zeros(6);
  m.inputs["time_step"] = zeros(1);
  m.outputs["actions"] = zeros(1);
  m.outputs["joint_pos"] = zeros(2);
  m.outputs["joint_vel"] = zeros(2);
  return m;
}

void set(DataStore & s, const std::string & name, double v, double stamp = 0.0)
{
  s.set(s.index(name), v, stamp);
}

void set_upright_robot(DataStore & s, double pos_a, double pos_b)
{
  set(s, "imu/orientation.w", 1.0);
  for (const auto * n : {"imu/orientation.x", "imu/orientation.y", "imu/orientation.z",
      "imu/angular_velocity.y", "imu/angular_velocity.z", "a/velocity",
      "cmd_vel/linear.y", "cmd_vel/angular.z"})
  {
    set(s, n, 0.0);
  }
  set(s, "imu/angular_velocity.x", 0.5);
  set(s, "a/position", pos_a);
  set(s, "b/position", pos_b);
  set(s, "b/velocity", 3.0);
  set(s, "cmd_vel/linear.x", 0.4);
}

}  // namespace

TEST(MjlabPreprocessor, RegistersTheEntriesItReads)
{
  const auto model = twist_model(1);
  DataStore state;
  MjlabPreprocessor pre;
  pre.configure(make_context(), model, state);
  for (const auto * n : {"imu/orientation.w", "imu/angular_velocity.x", "a/position",
      "b/velocity", "cmd_vel/linear.x", "cmd_vel/angular.z"})
  {
    EXPECT_TRUE(state.find(n).has_value()) << n;
  }
  EXPECT_FALSE(state.find("imu/linear_acceleration.x").has_value());  // base_lin_acc unused
}

TEST(MjlabPreprocessor, WaitsUntilEveryInputWasReceived)
{
  const auto model = twist_model(1);
  DataStore state;
  MjlabPreprocessor pre;
  pre.configure(make_context(), model, state);
  pre.reset();
  TensorMap in = model.inputs;
  EXPECT_FALSE(pre.compute(state, model.outputs, in));
}

TEST(MjlabPreprocessor, BuildsTermMajorHistoryOldestFirst)
{
  const auto model = twist_model(2);
  DataStore state;
  MjlabPreprocessor pre;
  pre.configure(make_context(), model, state);
  pre.reset();
  set_upright_robot(state, 1.1, 1.2);
  TensorMap in = model.inputs;
  TensorMap out = model.outputs;

  ASSERT_TRUE(pre.compute(state, out, in));
  // Like mjlab's CircularBuffer, the first step after reset back-fills the whole history with
  // the current value; the newest entry is last.
  const Floats first = {
    0.5F, 0, 0, 0.5F, 0, 0,       // base_ang_vel
    1.0F, 1.0F, 1.0F, 1.0F,       // joint_pos relative to the defaults
    0, 3.0F, 0, 3.0F,             // joint_vel
    0, 0,                         // actions (none yet)
    0.4F, 0, 0, 0.4F, 0, 0,       // command: lin x, lin y, ang z
    0, 0, -1.0F, 0, 0, -1.0F,     // imu_projected_gravity
  };
  for (size_t i = 0; i < first.size(); ++i) {
    EXPECT_NEAR(in["obs"].data[i], first[i], 1e-6) << "index " << i;
  }

  out["actions"].data[0] = 0.7F;  // raw (unscaled) action of the previous step
  set(state, "a/position", 2.1);
  ASSERT_TRUE(pre.compute(state, out, in));
  const Floats joint_pos = {1.0F, 1.0F, 2.0F, 1.0F};
  const Floats actions = {0.0F, 0.7F};
  for (size_t i = 0; i < 4; ++i) {EXPECT_NEAR(in["obs"].data[6 + i], joint_pos[i], 1e-6);}
  for (size_t i = 0; i < 2; ++i) {EXPECT_NEAR(in["obs"].data[14 + i], actions[i], 1e-6);}

  // reset() (policy activated again after a switch) keeps the history...
  set(state, "a/position", 3.1);
  pre.reset();
  ASSERT_TRUE(pre.compute(state, model.outputs, in));
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(in["obs"].data[6 + i], (Floats{2.0F, 1.0F, 3.0F, 1.0F})[i], 1e-6);
  }
  // ...clear() forgets it: the next step back-fills from the current value again.
  pre.clear();
  pre.reset();
  ASSERT_TRUE(pre.compute(state, model.outputs, in));
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(in["obs"].data[6 + i], (Floats{3.0F, 1.0F, 3.0F, 1.0F})[i], 1e-6);
  }
}

TEST(MjlabPreprocessor, ReportsTheOldestStampOfEachTerm)
{
  const auto model = twist_model(1);
  DataStore state;
  MjlabPreprocessor pre;
  pre.configure(make_context(), model, state);
  set_upright_robot(state, 1.1, 1.2);  // stamps 0
  for (const auto * n : {"imu/orientation.w", "imu/orientation.x", "imu/orientation.y",
      "imu/orientation.z"})
  {
    set(state, n, n[12] == 'w' ? 1.0 : 0.0, 5.0);
  }
  for (const auto * n : {"imu/angular_velocity.x", "imu/angular_velocity.y",
      "imu/angular_velocity.z"})
  {
    set(state, n, 0.0, 5.1);
  }
  set(state, "a/position", 1.1, 4.0);
  set(state, "b/position", 1.2, 4.2);
  set(state, "a/velocity", 0.0, 4.1);
  set(state, "b/velocity", 0.0, 4.1);
  for (const auto * n : {"cmd_vel/linear.x", "cmd_vel/linear.y", "cmd_vel/angular.z"}) {
    set(state, n, 0.0, 3.0);
  }
  pre.reset();
  TensorMap in = model.inputs;
  ASSERT_TRUE(pre.compute(state, model.outputs, in));
  const auto & stamps = pre.input_stamps();
  ASSERT_EQ(stamps.size(), 6u);  // one per term, in observation_names order
  const std::vector<std::pair<std::string, double>> expected = {
    {"base_ang_vel", 5.1}, {"joint_pos", 4.0}, {"joint_vel", 4.1},
    {"actions", 0.0},  // previous outputs: none yet
    {"command", 3.0}, {"imu_projected_gravity", 5.0}};
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(stamps[i].name, expected[i].first);
    EXPECT_DOUBLE_EQ(stamps[i].stamp, expected[i].second) << expected[i].first;
  }

  // The previous action derives from the previous observation, whose oldest input was 3.0.
  set(state, "cmd_vel/linear.x", 0.0, 6.0);
  set(state, "cmd_vel/linear.y", 0.0, 6.0);
  set(state, "cmd_vel/angular.z", 0.0, 6.0);
  ASSERT_TRUE(pre.compute(state, model.outputs, in));
  EXPECT_DOUBLE_EQ(pre.input_stamps()[3].stamp, 3.0);
  EXPECT_DOUBLE_EQ(pre.input_stamps()[4].stamp, 6.0);
}

TEST(MjlabPreprocessor, MotionCommandIsSeededFromRobotThenFedFromOutputs)
{
  const auto model = motion_model();
  DataStore state;
  MjlabPreprocessor pre;
  pre.configure(make_context(), model, state);
  EXPECT_FALSE(state.find("cmd_vel/linear.x").has_value());
  set(state, "a/position", 1.1);
  set(state, "b/position", 1.2);
  set(state, "a/velocity", 0.0);
  set(state, "b/velocity", 3.0);
  pre.reset();
  TensorMap in = model.inputs;
  TensorMap out = model.outputs;

  ASSERT_TRUE(pre.compute(state, out, in));
  // command = robot joint_pos (absolute), joint_vel; then joint_pos relative to the defaults.
  const Floats seeded = {1.1F, 1.2F, 0.0F, 3.0F, 1.0F, 1.0F};
  for (size_t i = 0; i < seeded.size(); ++i) {EXPECT_NEAR(in["obs"].data[i], seeded[i], 1e-6);}
  EXPECT_FLOAT_EQ(in["time_step"].data[0], 0.0F);

  out["joint_pos"].data = {5.0F, 6.0F};
  out["joint_vel"].data = {7.0F, 8.0F};
  ASSERT_TRUE(pre.compute(state, out, in));
  const Floats fed = {5.0F, 6.0F, 7.0F, 8.0F};
  for (size_t i = 0; i < fed.size(); ++i) {EXPECT_NEAR(in["obs"].data[i], fed[i], 1e-6);}
  EXPECT_FLOAT_EQ(in["time_step"].data[0], 1.0F);

  pre.reset();
  ASSERT_TRUE(pre.compute(state, out, in));
  EXPECT_NEAR(in["obs"].data[0], 1.1F, 1e-6);
  EXPECT_FLOAT_EQ(in["time_step"].data[0], 0.0F);
}

TEST(MjlabPreprocessor, RejectsInconsistentModels)
{
  DataStore state;
  MjlabPreprocessor pre;
  auto bad_dim = twist_model(1);
  bad_dim.inputs["obs"] = zeros(15);
  EXPECT_THROW(pre.configure(make_context(), bad_dim, state), std::runtime_error);

  auto unknown_term = twist_model(1);
  unknown_term.metadata["observation_names"] = "base_ang_vel,height_scan";
  EXPECT_THROW(pre.configure(make_context(), unknown_term, state), std::runtime_error);
}

TEST(MjlabPreprocessor, TermPluginReplacesABuiltinTerm)
{
  // command from TwistCommandTerm (topic, zero without message) instead of cmd_vel entries.
  const auto model = twist_model(1);
  DataStore state;
  MjlabPreprocessor pre;
  pre.configure(
    make_context({{"p.terms.command.type", "policy_framework_mjlab/TwistCommandTerm"},
      {"p.terms.command.topic", "/test_pre_cmd_vel"}}), model, state);
  EXPECT_FALSE(state.find("cmd_vel/linear.x").has_value());
  set(state, "imu/orientation.w", 1.0);
  for (const auto * n : {"imu/orientation.x", "imu/orientation.y", "imu/orientation.z",
      "imu/angular_velocity.x", "imu/angular_velocity.y", "imu/angular_velocity.z",
      "a/position", "b/position", "a/velocity", "b/velocity"})
  {
    set(state, n, 0.0);
  }
  pre.reset();
  TensorMap in = model.inputs;
  const double before = make_context().node->now().seconds();
  ASSERT_TRUE(pre.compute(state, model.outputs, in));
  for (size_t i = 8; i < 11; ++i) {EXPECT_FLOAT_EQ(in["obs"].data[i], 0.0F);}  // command
  EXPECT_GE(pre.input_stamps()[4].stamp, before);  // stamp from the plugin: zero made now
  EXPECT_FLOAT_EQ(in["obs"].data[13], -1.0F);  // gravity z still after it
}

TEST(MjlabPreprocessor, TermPluginProvidesATermUnknownToThePreprocessor)
{
  auto model = twist_model(2);
  model.metadata["observation_names"] = "base_ang_vel,my_velocity";
  model.inputs["obs"] = zeros(2 * (3 + 3));
  DataStore state;
  MjlabPreprocessor pre;
  pre.configure(
    make_context({{"p.terms.my_velocity.type", "policy_framework_mjlab/TwistCommandTerm"}}),
    model, state);
  EXPECT_EQ(pre.history_length(), 2U);
}

TEST(ProjectedGravity, RotatesWorldGravityIntoBodyFrame)
{
  float g[3];
  policy_framework_mjlab::projected_gravity(1.0, 0.0, 0.0, 0.0, g);
  EXPECT_NEAR(g[2], -1.0F, 1e-6);
  // Rolled +90 deg about x: gravity points along -y in the body frame.
  const double h = std::sqrt(0.5);
  policy_framework_mjlab::projected_gravity(h, h, 0.0, 0.0, g);
  EXPECT_NEAR(g[0], 0.0F, 1e-6);
  EXPECT_NEAR(g[1], -1.0F, 1e-6);
  EXPECT_NEAR(g[2], 0.0F, 1e-6);
  // Non-normalized quaternions are normalized.
  policy_framework_mjlab::projected_gravity(2.0, 0.0, 0.0, 0.0, g);
  EXPECT_NEAR(g[2], -1.0F, 1e-6);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
