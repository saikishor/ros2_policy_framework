// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "policy_framework/generic_stages.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::DataStore;
using policy_framework::ModelInfo;
using policy_framework::PluginContext;
using policy_framework::TensorMap;

namespace
{

PluginContext make_context(const std::vector<rclcpp::Parameter> & params)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  return {std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_generic", opts), "p"};
}

ModelInfo make_model()
{
  ModelInfo m;
  m.inputs["obs"] = {{1, 3}, std::vector<float>(3, 0.0F)};
  m.outputs["actions"] = {{1, 2}, std::vector<float>(2, 0.0F)};
  return m;
}

}  // namespace

TEST(ConcatPreprocessor, RegistersEntriesAndConcatenatesThem)
{
  const auto model = make_model();
  DataStore state;
  policy_framework::ConcatPreprocessor pre;
  pre.configure(
    make_context({{"p.input_tensor", "obs"},
        {"p.input_entries", std::vector<std::string>{"a", "b", "c"}}}), model, state);
  EXPECT_EQ(state.size(), 3u);

  TensorMap in = model.inputs;
  EXPECT_FALSE(pre.compute(state, model.outputs, in));  // waits for data
  state.set(state.index("a"), 1.0, 0.0);
  state.set(state.index("b"), 2.0, 0.0);
  state.set(state.index("c"), 3.0, 0.0);
  ASSERT_TRUE(pre.compute(state, model.outputs, in));
  EXPECT_EQ(in["obs"].data, (std::vector<float>{1.0F, 2.0F, 3.0F}));
}

TEST(ConcatPreprocessor, ReportsTheStampOfEachEntry)
{
  const auto model = make_model();
  DataStore state;
  policy_framework::ConcatPreprocessor pre;
  pre.configure(
    make_context({{"p.input_entries", std::vector<std::string>{"a", "b", "c"}}}), model, state);
  state.set(state.index("a"), 1.0, 10.0);
  state.set(state.index("b"), 2.0, 11.0);
  state.set(state.index("c"), 3.0, 12.5);
  TensorMap in = model.inputs;
  ASSERT_TRUE(pre.compute(state, model.outputs, in));
  const auto & stamps = pre.input_stamps();
  ASSERT_EQ(stamps.size(), 3u);
  EXPECT_EQ(stamps[0].name, "a");
  EXPECT_DOUBLE_EQ(stamps[0].stamp, 10.0);
  EXPECT_EQ(stamps[2].name, "c");
  EXPECT_DOUBLE_EQ(stamps[2].stamp, 12.5);
}

TEST(ConcatPreprocessor, RejectsSizeMismatch)
{
  DataStore state;
  policy_framework::ConcatPreprocessor pre;
  EXPECT_THROW(
    pre.configure(
      make_context({{"p.input_tensor", "obs"},
          {"p.input_entries", std::vector<std::string>{"a"}}}), make_model(), state),
    std::runtime_error);
}

TEST(SplitPostprocessor, WritesEachOutputValueToItsEntry)
{
  const auto model = make_model();
  DataStore commands;
  policy_framework::SplitPostprocessor post;
  post.configure(
    make_context({{"p.output_tensor", "actions"},
        {"p.output_entries", std::vector<std::string>{"j1/position", "j2/position"}}}),
    model, commands);
  TensorMap out = model.outputs;
  out["actions"].data = {0.5F, -0.5F};
  ASSERT_TRUE(post.compute(out, 7.5, commands));
  EXPECT_DOUBLE_EQ(commands.get(commands.index("j1/position")), 0.5);
  EXPECT_DOUBLE_EQ(commands.stamp(commands.index("j2/position")), 7.5);
  EXPECT_DOUBLE_EQ(commands.get(commands.index("j2/position")), -0.5);
}

TEST(NullStages, MakeAPolicyWithoutAModel)
{
  policy_framework::NullInference inference;
  inference.configure(make_context({}));
  EXPECT_TRUE(inference.model().inputs.empty());
  EXPECT_TRUE(inference.model().outputs.empty());
  DataStore state;
  policy_framework::NullPreprocessor pre;
  pre.configure(make_context({}), inference.model(), state);
  EXPECT_EQ(state.size(), 0u);
  TensorMap in, out;
  EXPECT_TRUE(pre.compute(state, out, in));  // never waits
  EXPECT_TRUE(inference.run(in, out));
}

TEST(FixedPosePostprocessor, CommandsThePoseWithZeroVelocityAndItsGains)
{
  DataStore commands;
  policy_framework::FixedPosePostprocessor post;
  post.configure(
    make_context({{"p.joints", std::vector<std::string>{"j1", "j2"}},
        {"p.positions", std::vector<double>{0.5, -1.0}},
        {"p.stiffness", std::vector<double>{40.0, 60.0}},
        {"p.damping", std::vector<double>{2.0, 3.0}}}), ModelInfo(), commands);
  auto value = [&](const std::string & n) {return commands.get(commands.index(n));};

  // The gains are known before the first step, so that an executor can hold the robot.
  post.initial_commands(commands);
  EXPECT_DOUBLE_EQ(value("j2/stiffness"), 60.0);
  EXPECT_DOUBLE_EQ(value("j1/damping"), 2.0);
  EXPECT_FALSE(commands.received(commands.index("j1/position")));

  ASSERT_TRUE(post.compute(TensorMap(), 7.0, commands));
  EXPECT_DOUBLE_EQ(value("j1/position"), 0.5);
  EXPECT_DOUBLE_EQ(value("j2/position"), -1.0);
  EXPECT_TRUE(commands.received(commands.index("j2/velocity")));
  EXPECT_DOUBLE_EQ(value("j2/velocity"), 0.0);
  EXPECT_DOUBLE_EQ(commands.stamp(commands.index("j1/position")), 7.0);
}

TEST(FixedPosePostprocessor, WritesNoGainsUnlessConfigured)
{
  DataStore commands;
  policy_framework::FixedPosePostprocessor post;
  post.configure(
    make_context({{"p.joints", std::vector<std::string>{"j1"}},
        {"p.positions", std::vector<double>{0.5}}}), ModelInfo(), commands);
  EXPECT_FALSE(commands.find("j1/stiffness").has_value());
}

TEST(FixedPosePostprocessor, RejectsArraysThatDoNotMatchTheJoints)
{
  DataStore commands;
  policy_framework::FixedPosePostprocessor post;
  EXPECT_THROW(
    post.configure(
      make_context({{"p.joints", std::vector<std::string>{"j1", "j2"}},
          {"p.positions", std::vector<double>{0.5}}}), ModelInfo(), commands),
    std::runtime_error);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
