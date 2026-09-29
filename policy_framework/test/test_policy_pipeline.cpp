// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "policy_framework/policy_pipeline.hpp"

using policy_framework::DataStore;
using policy_framework::ModelInfo;
using policy_framework::PluginContext;
using policy_framework::Policy;
using policy_framework::PolicyPipeline;
using policy_framework::TensorMap;

namespace
{

// actions = obs + offset
class FakeInference : public policy_framework::Inference
{
public:
  explicit FakeInference(float offset)
  : offset_(offset)
  {
    model_.inputs["obs"] = {{1, 1}, {0.0F}};
    model_.outputs["actions"] = {{1, 1}, {0.0F}};
  }
  void configure(const PluginContext &) override {}
  const ModelInfo & model() const override {return model_;}
  bool run(const TensorMap & in, TensorMap & out) override
  {
    ++runs;
    out.at("actions").data[0] = in.at("obs").data[0] + offset_;
    return true;
  }
  int runs = 0;

private:
  ModelInfo model_;
  float offset_;
};

// obs = previous action, so consecutive steps accumulate the inference offset.
class FakePreprocessor : public policy_framework::Preprocessor
{
public:
  void configure(const PluginContext &, const ModelInfo &, DataStore &) override {}
  void reset() override {++resets;}
  bool compute(const DataStore &, const TensorMap & last, TensorMap & in) override
  {
    if (!ready) {return false;}
    in.at("obs").data[0] = last.at("actions").data[0];
    return true;
  }
  bool ready = true;
  int resets = 0;
};

class FakePostprocessor : public policy_framework::Postprocessor
{
public:
  void configure(const PluginContext &, const ModelInfo &, DataStore &) override {}
  bool compute(const TensorMap & out, DataStore & commands) override
  {
    commands.set(commands.index("cmd"), out.at("actions").data[0], 0.0);
    return true;
  }
};

struct Fakes
{
  std::shared_ptr<FakePreprocessor> pre = std::make_shared<FakePreprocessor>();
  std::shared_ptr<FakeInference> inf;
  std::shared_ptr<FakePostprocessor> post = std::make_shared<FakePostprocessor>();

  explicit Fakes(float offset)
  : inf(std::make_shared<FakeInference>(offset)) {}

  Policy policy(const std::string & name, int max_steps = 0, bool interruptible = true) const
  {
    Policy p;
    p.name = name;
    p.preprocessor = pre;
    p.inference = inf;
    p.postprocessor = post;
    p.max_steps = max_steps;
    p.interruptible = interruptible;
    return p;
  }
};

class PolicyPipelineTest : public ::testing::Test
{
protected:
  void SetUp() override {commands.add("cmd");}
  double cmd() const {return commands.get(commands.index("cmd"));}

  DataStore state;
  DataStore commands;
  PolicyPipeline pipeline;
};

}  // namespace

TEST_F(PolicyPipelineTest, StepRunsStagesAndFeedsBackPreviousOutputs)
{
  Fakes a(1.0F);
  pipeline.add_policy(a.policy("a"));

  ASSERT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::OK);
  EXPECT_DOUBLE_EQ(cmd(), 1.0);
  ASSERT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::OK);
  EXPECT_DOUBLE_EQ(cmd(), 2.0);
  EXPECT_EQ(a.pre->resets, 1);  // reset once on the first activation
}

TEST_F(PolicyPipelineTest, StepWaitsWhenPreprocessorHasNoData)
{
  Fakes a(1.0F);
  a.pre->ready = false;
  pipeline.add_policy(a.policy("a"));
  EXPECT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::WAITING_FOR_DATA);
  EXPECT_EQ(a.inf->runs, 0);
}

TEST_F(PolicyPipelineTest, StepWithoutPoliciesIsAnError)
{
  EXPECT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::ERROR);
}

TEST_F(PolicyPipelineTest, SwitchIsAppliedOnNextStepAndResetsTheNewPolicy)
{
  Fakes a(1.0F), b(10.0F);
  pipeline.add_policy(a.policy("a"));
  pipeline.add_policy(b.policy("b"));
  pipeline.step(state, commands);
  pipeline.step(state, commands);

  std::string msg;
  ASSERT_TRUE(pipeline.request_switch("b", msg)) << msg;
  EXPECT_EQ(pipeline.active_policy(), "a");  // not applied before the next step
  pipeline.step(state, commands);
  EXPECT_EQ(pipeline.active_policy(), "b");
  EXPECT_DOUBLE_EQ(cmd(), 10.0);  // b starts from zeroed outputs
  EXPECT_EQ(b.pre->resets, 1);

  // Switching back resets 'a' as well: its feedback starts from zero again.
  ASSERT_TRUE(pipeline.request_switch("a", msg));
  pipeline.step(state, commands);
  EXPECT_DOUBLE_EQ(cmd(), 1.0);
  EXPECT_EQ(a.pre->resets, 2);
}

TEST_F(PolicyPipelineTest, SwitchToUnknownPolicyIsRejected)
{
  Fakes a(1.0F);
  pipeline.add_policy(a.policy("a"));
  std::string msg;
  EXPECT_FALSE(pipeline.request_switch("nope", msg));
  EXPECT_NE(msg.find("nope"), std::string::npos);
}

TEST_F(PolicyPipelineTest, NonInterruptiblePolicyRejectsSwitchAndReturnsToDefaultAfterMaxSteps)
{
  Fakes a(1.0F), jump(10.0F);
  pipeline.add_policy(a.policy("a"));
  pipeline.add_policy(jump.policy("jump", 2, false));
  std::string msg;
  ASSERT_TRUE(pipeline.request_switch("jump", msg));
  pipeline.step(state, commands);
  EXPECT_EQ(pipeline.active_policy(), "jump");
  EXPECT_FALSE(pipeline.request_switch("a", msg));

  pipeline.step(state, commands);  // second (= max) step of 'jump'
  EXPECT_EQ(pipeline.active_policy(), "jump");
  pipeline.step(state, commands);  // auto-return applied
  EXPECT_EQ(pipeline.active_policy(), "a");
  EXPECT_TRUE(pipeline.request_switch("jump", msg));
}

TEST_F(PolicyPipelineTest, AddPolicyRejectsDuplicateNames)
{
  Fakes a(1.0F);
  pipeline.add_policy(a.policy("a"));
  EXPECT_THROW(pipeline.add_policy(a.policy("a")), std::invalid_argument);
}

TEST_F(PolicyPipelineTest, RestartActivatesDefaultPolicyEvenWhenNotInterruptible)
{
  Fakes a(1.0F), jump(10.0F);
  pipeline.add_policy(a.policy("a"));
  pipeline.add_policy(jump.policy("jump", 0, false));
  std::string msg;
  ASSERT_TRUE(pipeline.request_switch("jump", msg));
  pipeline.step(state, commands);
  ASSERT_EQ(pipeline.active_policy(), "jump");

  pipeline.restart();
  pipeline.step(state, commands);
  EXPECT_EQ(pipeline.active_policy(), "a");
  EXPECT_EQ(a.pre->resets, 1);
  EXPECT_DOUBLE_EQ(cmd(), 1.0);
}
