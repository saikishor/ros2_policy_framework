// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "policy_framework/policy_replay.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using policy_framework::ModelInfo;
using policy_framework::PluginContext;
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
    model_.inputs["obs"] = {{1, 1}, {0.0F}};
    model_.outputs["actions"] = {{1, 1}, {0.0F}};
  }
  void configure(const PluginContext &) override {}
  const ModelInfo & model() const override {return model_;}
  bool run(const TensorMap & in, TensorMap & out) override
  {
    out.at("actions").data[0] = 2 * in.at("obs").data[0];
    return true;
  }

private:
  ModelInfo model_;
};

PolicyStep step(uint64_t n, float obs, float action)
{
  PolicyStep msg;
  msg.policy = "walk";
  msg.step = n;
  msg.status = PolicyStep::STATUS_OK;
  Tensor in, out;
  in.name = "obs";
  in.shape = {1, 1};
  in.data = {obs};
  out.name = "actions";
  out.shape = {1, 1};
  out.data = {action};
  msg.inputs = {in};
  msg.outputs = {out};
  return msg;
}

class PolicyReplayTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    bag = (std::filesystem::temp_directory_path() /
      ("policy_replay_test_" + std::to_string(::getpid()))).string();
    std::filesystem::remove_all(bag);
  }
  void TearDown() override {std::filesystem::remove_all(bag);}

  std::string bag;
};

}  // namespace

TEST_F(PolicyReplayTest, ReplaysThePolicyStepTopicOfABag)
{
  {
    rosbag2_cpp::Writer writer;
    writer.open(bag);
    writer.write(sensor_msgs::msg::JointState(), "/joint_states", rclcpp::Time(1, 0));
    writer.write(step(1, 1.0F, 2.0F), "/policy_controller/policy_step", rclcpp::Time(2, 0));
    writer.write(step(2, 2.0F, 4.25F), "/policy_controller/policy_step", rclcpp::Time(3, 0));
  }
  // The only PolicyStep topic of the bag is found without naming it.
  const auto summary = policy_framework::replay_bag(bag, "", std::make_shared<DoubleInference>());
  EXPECT_EQ(summary.replayed, 2u);
  EXPECT_DOUBLE_EQ(summary.outputs.at("actions").max_error, 0.25);
  EXPECT_EQ(summary.outputs.at("actions").worst_step, 2u);
}

TEST_F(PolicyReplayTest, NeedsTheTopicWhenTheBagHasSeveralPolicyStepTopics)
{
  {
    rosbag2_cpp::Writer writer;
    writer.open(bag);
    writer.write(step(1, 1.0F, 2.0F), "/a/policy_step", rclcpp::Time(1, 0));
    writer.write(step(1, 1.0F, 3.0F), "/b/policy_step", rclcpp::Time(2, 0));
  }
  EXPECT_THROW(
    policy_framework::replay_bag(bag, "", std::make_shared<DoubleInference>()),
    std::runtime_error);
  const auto summary =
    policy_framework::replay_bag(bag, "/b/policy_step", std::make_shared<DoubleInference>());
  EXPECT_EQ(summary.replayed, 1u);
  EXPECT_DOUBLE_EQ(summary.outputs.at("actions").max_error, 1.0);
}

TEST_F(PolicyReplayTest, FailsWithoutPolicySteps)
{
  {
    rosbag2_cpp::Writer writer;
    writer.open(bag);
    writer.write(sensor_msgs::msg::JointState(), "/joint_states", rclcpp::Time(1, 0));
  }
  EXPECT_THROW(
    policy_framework::replay_bag(bag, "", std::make_shared<DoubleInference>()),
    std::runtime_error);
  EXPECT_THROW(
    policy_framework::replay_bag(bag, "/joint_states", std::make_shared<DoubleInference>()),
    std::runtime_error);
}
