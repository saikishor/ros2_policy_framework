// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "policy_framework/core/step_replayer.hpp"

using policy_framework::ModelInfo;
using policy_framework::PluginContext;
using policy_framework::StepReplayer;
using policy_framework::TensorMap;
using policy_framework::msg::PolicyStep;
using policy_framework::msg::Tensor;

namespace
{

// actions = 2 * obs
class DoubleInference : public policy_framework::Inference
{
public:
  DoubleInference()
  {
    model_.inputs["obs"] = {{1, 2}, {0.0F, 0.0F}};
    model_.outputs["actions"] = {{1, 2}, {0.0F, 0.0F}};
  }
  void configure(const PluginContext &) override {}
  const ModelInfo & model() const override {return model_;}
  bool run(const TensorMap & in, TensorMap & out) override
  {
    for (size_t i = 0; i < 2; ++i) {out.at("actions").data[i] = 2 * in.at("obs").data[i];}
    return true;
  }

private:
  ModelInfo model_;
};

Tensor tensor(const std::string & name, std::vector<float> data)
{
  Tensor t;
  t.name = name;
  t.shape = {1, static_cast<int64_t>(data.size())};
  t.data = std::move(data);
  return t;
}

PolicyStep recorded(std::vector<float> obs, std::vector<float> actions, uint64_t step = 1)
{
  PolicyStep msg;
  msg.policy = "walk";
  msg.step = step;
  msg.status = PolicyStep::STATUS_OK;
  msg.inputs = {tensor("obs", std::move(obs))};
  msg.outputs = {tensor("actions", std::move(actions))};
  return msg;
}

}  // namespace

TEST(StepReplayer, ReportsTheLargestDifferenceOfEveryOutput)
{
  StepReplayer replayer(std::make_shared<DoubleInference>());
  ASSERT_TRUE(replayer.replay(recorded({1.0F, 2.0F}, {2.0F, 4.0F})));
  ASSERT_TRUE(replayer.replay(recorded({1.0F, 2.0F}, {2.0F, 4.5F}, 2)));
  ASSERT_TRUE(replayer.replay(recorded({0.0F, 0.0F}, {0.25F, 0.0F}, 3)));

  const auto & summary = replayer.summary();
  EXPECT_EQ(summary.replayed, 3u);
  EXPECT_EQ(summary.skipped, 0u);
  ASSERT_EQ(summary.outputs.size(), 1u);
  const auto & actions = summary.outputs.at("actions");
  EXPECT_DOUBLE_EQ(actions.max_error, 0.5);
  EXPECT_EQ(actions.worst_step, 2u);
  EXPECT_EQ(actions.worst_policy, "walk");
  EXPECT_EQ(actions.steps, 3u);
  EXPECT_NEAR(actions.mean_error, (0.0 + 0.5 + 0.25) / 3, 1e-9);
}

TEST(StepReplayer, SkipsStepsWhoseModelDidNotRun)
{
  StepReplayer replayer(std::make_shared<DoubleInference>());
  auto waiting = recorded({1.0F, 2.0F}, {0.0F, 0.0F});
  waiting.status = PolicyStep::STATUS_WAITING_FOR_DATA;
  EXPECT_FALSE(replayer.replay(waiting));
  EXPECT_EQ(replayer.summary().replayed, 0u);
  EXPECT_EQ(replayer.summary().skipped, 1u);
}

TEST(StepReplayer, ReplaysOnlyTheSelectedPolicy)
{
  StepReplayer replayer(std::make_shared<DoubleInference>(), "jump");
  EXPECT_FALSE(replayer.replay(recorded({1.0F, 2.0F}, {2.0F, 4.0F})));
  EXPECT_EQ(replayer.summary().skipped, 1u);
}

TEST(StepReplayer, RejectsStepsThatDoNotMatchTheModel)
{
  StepReplayer replayer(std::make_shared<DoubleInference>());
  EXPECT_THROW(replayer.replay(recorded({1.0F, 2.0F, 3.0F}, {2.0F, 4.0F})), std::runtime_error);
  auto renamed = recorded({1.0F, 2.0F}, {2.0F, 4.0F});
  renamed.inputs[0].name = "observation";
  EXPECT_THROW(replayer.replay(renamed), std::runtime_error);
}

TEST(StepReplayer, OutputsTheModelDoesNotHaveAreIgnored)
{
  StepReplayer replayer(std::make_shared<DoubleInference>());
  auto msg = recorded({1.0F, 2.0F}, {2.0F, 4.0F});
  msg.outputs.push_back(tensor("echo", {7.0F}));
  ASSERT_TRUE(replayer.replay(msg));
  EXPECT_EQ(replayer.summary().outputs.count("echo"), 0u);
}
