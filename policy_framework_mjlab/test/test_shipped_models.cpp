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

// Runs every ONNX model found under $PAL_POLICY_MODELS_DIR (e.g. the `models` directory of
// pal_policy_deployer) in a `*_mjlab` directory through the mjlab stages, as deployed on Kangaroo. Skipped when unset.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "policy_framework/onnx_inference.hpp"
#include "policy_framework/policy_pipeline.hpp"
#include "policy_framework_mjlab/metadata.hpp"
#include "policy_framework_mjlab/mjlab_postprocessor.hpp"
#include "policy_framework_mjlab/mjlab_preprocessor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fs = std::filesystem;
using policy_framework::DataStore;
using policy_framework::PolicyPipeline;

TEST(ShippedModels, EveryModelRunsThroughTheMjlabStages)
{
  const char * dir = std::getenv("PAL_POLICY_MODELS_DIR");
  if (dir == nullptr) {
    GTEST_SKIP() << "PAL_POLICY_MODELS_DIR not set";
  }
  // History lengths derived offline from the ONNX input sizes and metadata.
  const std::map<std::string, size_t> expected_history = {
    {"loco_v619_mjlab", 3}, {"loco_v630_mjlab", 3}, {"loco_v636_mjlab", 3},
    {"loco_v639_mjlab", 3}, {"loco_v700_mjlab", 3}};

  size_t tested = 0;
  for (const auto & entry : fs::recursive_directory_iterator(dir)) {
    const auto model_dir = entry.path().parent_path().filename().string();
    // Only mjlab policies (the directory also holds e.g. non-mjlab TIAGo models).
    const std::string suffix = "_mjlab";
    if (entry.path().extension() != ".onnx" || model_dir.size() < suffix.size() ||
      model_dir.compare(model_dir.size() - suffix.size(), suffix.size(), suffix) != 0)
    {
      continue;
    }
    SCOPED_TRACE(model_dir);
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
      {"p.model_path", entry.path().string()},
      {"p.imu_name", "torso_imu_sensor"},
      {"p.excluded_joints", std::vector<std::string>{
          "leg_left_femur_joint", "leg_left_knee_joint",
          "leg_right_femur_joint", "leg_right_knee_joint"}},
    });
    const policy_framework::PluginContext ctx{
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("shipped_" + std::to_string(tested), opts),
      "p"};

    auto inference = std::make_shared<policy_framework::OnnxInference>();
    inference->configure(ctx);
    DataStore state, commands;
    auto pre = std::make_shared<policy_framework_mjlab::MjlabPreprocessor>();
    pre->configure(ctx, inference->model(), state);
    auto post = std::make_shared<policy_framework_mjlab::MjlabPostprocessor>();
    post->configure(ctx, inference->model(), commands);
    const auto it = expected_history.find(model_dir);
    EXPECT_EQ(pre->history_length(), it == expected_history.end() ? 1u : it->second);

    // Robot standing still at the default pose, upright.
    const auto joints = policy_framework_mjlab::split(inference->model().metadata.at("joint_names"));
    const auto defaults = policy_framework_mjlab::parse_doubles(
      inference->model().metadata.at("default_joint_pos"));
    for (size_t i = 0; i < state.size(); ++i) {state.set(i, 0.0, 0.0);}
    state.set(state.index("torso_imu_sensor/orientation.w"), 1.0, 0.0);
    for (size_t j = 0; j < joints.size(); ++j) {
      state.set(state.index(joints[j] + "/position"), defaults[j], 0.0);
    }

    PolicyPipeline pipeline;
    pipeline.add_policy({"p", pre, inference, post, 0, true});
    for (int step = 0; step < 5; ++step) {
      ASSERT_EQ(pipeline.step(state, commands), PolicyPipeline::Status::OK);
    }
    for (size_t i = 0; i < commands.size(); ++i) {
      EXPECT_TRUE(std::isfinite(commands.get(i))) << commands.names()[i];
    }
    ++tested;
  }
  EXPECT_GT(tested, 0u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
