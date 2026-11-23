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

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "policy_framework/onnx_inference.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::OnnxInference;
using policy_framework::PluginContext;

namespace
{

PluginContext make_context(const std::string & model_path)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"policy.model_path", model_path}});
  return {std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_onnx", opts), "policy"};
}

const std::string kModel = std::string(TEST_MODELS_DIR) + "/affine.onnx";

}  // namespace

TEST(OnnxInference, ExposesMetadataAndStaticShapes)
{
  OnnxInference inf;
  inf.configure(make_context(kModel));
  const auto & m = inf.model();
  EXPECT_EQ(m.metadata.at("joint_names"), "a,b");
  // Dynamic batch dimension is fixed to 1.
  EXPECT_EQ(m.inputs.at("obs").shape, (std::vector<int64_t>{1, 3}));
  EXPECT_EQ(m.inputs.at("obs").data.size(), 3u);
  EXPECT_EQ(m.inputs.at("time_step").shape, (std::vector<int64_t>{1, 1}));
  EXPECT_EQ(m.outputs.at("actions").shape, (std::vector<int64_t>{1, 2}));
  EXPECT_EQ(m.outputs.at("echo").data.size(), 1u);
}

TEST(OnnxInference, RunWritesOutputsInPlace)
{
  OnnxInference inf;
  inf.configure(make_context(kModel));
  auto in = inf.model().inputs;
  auto out = inf.model().outputs;
  in["obs"].data = {1.0F, 2.0F, 3.0F};
  in["time_step"].data = {7.0F};
  const float * actions_ptr = out["actions"].data.data();

  ASSERT_TRUE(inf.run(in, out));
  // [1,2,3] @ [[1,0],[0,2],[1,1]] + [0.5,-0.5] = [4.5, 6.5]
  EXPECT_FLOAT_EQ(out["actions"].data[0], 4.5F);
  EXPECT_FLOAT_EQ(out["actions"].data[1], 6.5F);
  EXPECT_FLOAT_EQ(out["echo"].data[0], 7.0F);
  EXPECT_EQ(out["actions"].data.data(), actions_ptr);  // no reallocation

  in["obs"].data = {0.0F, 0.0F, 0.0F};
  ASSERT_TRUE(inf.run(in, out));
  EXPECT_FLOAT_EQ(out["actions"].data[0], 0.5F);
}

TEST(OnnxInference, RejectsANonPositiveThreadCount)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"p.model_path", kModel}, {"p.num_threads", 0}});
  policy_framework::OnnxInference inf;
  EXPECT_ANY_THROW(
    inf.configure({std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_onnx_threads", opts),
        "p"}));
}

TEST(OnnxInference, ModelPathCanBeADirectory)
{
  OnnxInference inf;
  inf.configure(make_context(TEST_MODELS_DIR));
  EXPECT_EQ(inf.model().outputs.size(), 2u);
}

TEST(OnnxInference, MissingModelThrows)
{
  OnnxInference inf;
  EXPECT_ANY_THROW(inf.configure(make_context("/nonexistent/model.onnx")));
  EXPECT_ANY_THROW(inf.configure(make_context("")));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
