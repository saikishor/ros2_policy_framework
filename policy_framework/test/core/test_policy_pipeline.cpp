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
#include <memory>
#include <string>
#include <thread>

#include "policy_framework/core/policy_pipeline.hpp"

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
    std::this_thread::sleep_for(delay);
    out.at("actions").data[0] = nan ? std::nanf("") : in.at("obs").data[0] + offset_;
    return true;
  }
  int runs = 0;
  bool nan = false;  ///< Answer NaN, as a diverged model does.
  std::chrono::microseconds delay{0};  ///< Duration of each run.

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
  void clear() override {++clears;}
  void observe(const DataStore &, const TensorMap &) override {++observations;}
  bool compute(const DataStore &, const TensorMap & last, TensorMap & in) override
  {
    if (!ready) {return false;}
    in.at("obs").data[0] = last.at("actions").data[0];
    return true;
  }
  const policy_framework::InputStamps & input_stamps() const override {return stamps;}
  bool ready = true;
  policy_framework::InputStamps stamps;
  int resets = 0;
  int clears = 0;
  int observations = 0;
};

class FakePostprocessor : public policy_framework::Postprocessor
{
public:
  void configure(const PluginContext &, const ModelInfo &, DataStore &) override {}
  void initial_commands(DataStore & commands) override
  {
    commands.set(commands.index("cmd"), initial, 0.0);
  }
  double initial = 0.0;
  bool compute(const TensorMap & out, double stamp, DataStore & commands) override
  {
    commands.set(commands.index("cmd"), out.at("actions").data[0], stamp);
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
  EXPECT_EQ(a.pre->resets, 1);  // activated once
  EXPECT_EQ(a.pre->clears, 1);  // first start is from scratch
}

TEST_F(PolicyPipelineTest, InitialCommandsComeFromTheDefaultPolicy)
{
  Fakes a(1.0F), b(10.0F);
  a.post->initial = 3.0;
  b.post->initial = 5.0;
  pipeline.add_policy(a.policy("a"));
  pipeline.add_policy(b.policy("b"));
  pipeline.write_initial_commands(commands);
  EXPECT_DOUBLE_EQ(cmd(), 3.0);
}

TEST_F(PolicyPipelineTest, InactivePoliciesObserveEveryStep)
{
  Fakes a(1.0F), b(10.0F);
  pipeline.add_policy(a.policy("a"));
  pipeline.add_policy(b.policy("b"));

  for (int i = 0; i < 3; ++i) {ASSERT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::OK);}
  EXPECT_EQ(a.pre->observations, 0);  // the active policy computes instead
  EXPECT_EQ(b.pre->observations, 3);
  EXPECT_EQ(b.inf->runs, 0);

  std::string msg;
  ASSERT_TRUE(pipeline.request_switch("b", msg));
  ASSERT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::OK);
  EXPECT_EQ(a.pre->observations, 1);
  EXPECT_EQ(b.pre->observations, 3);
}

TEST_F(PolicyPipelineTest, CommandsCarryTheOldestStampOfTheObservation)
{
  Fakes a(1.0F);
  // Unknown (0) stamps and inputs fed back from previous outputs do not count.
  a.pre->stamps = {{"imu", 5.0, false}, {"joints", 3.0, false}, {"cmd", 0.0, false},
    {"actions", 1.0, true}};
  pipeline.add_policy(a.policy("a"));
  ASSERT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::OK);
  EXPECT_DOUBLE_EQ(pipeline.observation_stamp(), 3.0);
  EXPECT_DOUBLE_EQ(commands.stamp(commands.index("cmd")), 3.0);
  EXPECT_EQ(pipeline.input_stamps().size(), 4u);
}

TEST_F(PolicyPipelineTest, UserCommandsDoNotCountForTheObservationStamp)
{
  Fakes a(1.0F);
  // A held command is old by design (its own timeout handles a stopped operator): it must not
  // make the observation look old, nor hide a stale sensor.
  a.pre->stamps = {{"imu", 5.0, false, false}, {"motion_reference", 1.0, false, true}};
  pipeline.add_policy(a.policy("a"));
  ASSERT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::OK);
  EXPECT_DOUBLE_EQ(pipeline.observation_stamp(), 5.0);
  EXPECT_DOUBLE_EQ(pipeline.input_stamps()[1].stamp, 1.0);  // still reported
}

TEST_F(PolicyPipelineTest, StepWaitsWhenPreprocessorHasNoData)
{
  Fakes a(1.0F);
  a.pre->ready = false;
  pipeline.add_policy(a.policy("a"));
  EXPECT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::WAITING_FOR_DATA);
  EXPECT_EQ(a.inf->runs, 0);
}

TEST_F(PolicyPipelineTest, MeasuresTheDurationOfEachStage)
{
  Fakes a(1.0F);
  a.inf->delay = std::chrono::microseconds(2000);
  pipeline.add_policy(a.policy("a"));
  ASSERT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::OK);
  const auto & t = pipeline.stage_times();
  EXPECT_GE(t.inference_us, 2000.0);
  EXPECT_GT(t.preprocessor_us, 0.0);
  EXPECT_GT(t.postprocessor_us, 0.0);
  EXPECT_LT(t.preprocessor_us + t.postprocessor_us, t.inference_us);

  // Stages that did not run in the last step measure 0, not the previous step's duration.
  a.pre->ready = false;
  ASSERT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::WAITING_FOR_DATA);
  EXPECT_DOUBLE_EQ(pipeline.stage_times().inference_us, 0.0);
  EXPECT_DOUBLE_EQ(pipeline.stage_times().postprocessor_us, 0.0);
}

TEST_F(PolicyPipelineTest, NonFiniteOutputsAreAnErrorAndNeverReachTheCommands)
{
  Fakes a(1.0F);
  pipeline.add_policy(a.policy("a"));
  ASSERT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::OK);
  a.inf->nan = true;
  EXPECT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::ERROR);
  EXPECT_DOUBLE_EQ(cmd(), 1.0);  // the postprocessor did not run
  EXPECT_EQ(pipeline.error(), "policy 'a': model output 'actions' is not finite");
}

TEST_F(PolicyPipelineTest, StepWithoutPoliciesSaysWhyItFailed)
{
  EXPECT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::ERROR);
  EXPECT_EQ(pipeline.error(), "no policy loaded");
}

TEST_F(PolicyPipelineTest, StepWithoutPoliciesIsAnError)
{
  EXPECT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::ERROR);
}

TEST_F(PolicyPipelineTest, SwitchIsAppliedOnNextStepAndKeepsThePolicyMemory)
{
  Fakes a(1.0F), b(10.0F);
  pipeline.add_policy(a.policy("a"));
  pipeline.add_policy(b.policy("b"));
  pipeline.step(state, commands);
  pipeline.step(state, commands);  // a's last action: 2

  std::string msg;
  ASSERT_TRUE(pipeline.request_switch("b", msg)) << msg;
  EXPECT_EQ(pipeline.active_policy(), "a");  // not applied before the next step
  pipeline.step(state, commands);
  EXPECT_EQ(pipeline.active_policy(), "b");
  EXPECT_DOUBLE_EQ(cmd(), 10.0);  // b never ran: zero outputs
  EXPECT_EQ(b.pre->resets, 1);
  EXPECT_EQ(b.pre->clears, 1);  // first activation of b

  // Switching back activates 'a' again (reset) but keeps its memory: it continues from its
  // last action, like a policy that kept observing (no clear).
  ASSERT_TRUE(pipeline.request_switch("a", msg));
  pipeline.step(state, commands);
  EXPECT_DOUBLE_EQ(cmd(), 3.0);
  EXPECT_EQ(a.pre->resets, 2);
  EXPECT_EQ(a.pre->clears, 1);
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

TEST_F(PolicyPipelineTest, RestartActivatesDefaultPolicyFromScratchEvenWhenNotInterruptible)
{
  Fakes a(1.0F), jump(10.0F);
  pipeline.add_policy(a.policy("a"));
  pipeline.add_policy(jump.policy("jump", 0, false));
  pipeline.step(state, commands);
  pipeline.step(state, commands);  // a's last action: 2
  std::string msg;
  ASSERT_TRUE(pipeline.request_switch("jump", msg));
  pipeline.step(state, commands);
  ASSERT_EQ(pipeline.active_policy(), "jump");

  pipeline.restart();
  pipeline.step(state, commands);
  EXPECT_EQ(pipeline.active_policy(), "a");
  EXPECT_DOUBLE_EQ(cmd(), 1.0);  // outputs cleared
  EXPECT_EQ(a.pre->clears, 2);
  EXPECT_EQ(jump.pre->clears, 1);  // every policy starts from scratch after a restart
}
