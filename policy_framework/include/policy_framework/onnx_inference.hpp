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

#ifndef POLICY_FRAMEWORK__ONNX_INFERENCE_HPP_
#define POLICY_FRAMEWORK__ONNX_INFERENCE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "policy_framework/plugin_interfaces.hpp"

namespace policy_framework
{

/**
 * @brief Inference stage running an ONNX model with ONNX Runtime (CPU).
 *
 * Parameters (under `<policy>.`):
 * - `model_path` (string, required): `.onnx` file, or a directory whose first `.onnx` file (in
 *   lexicographic order) is used.
 * - `num_threads` (int, default 1): intra-op threads. 1 keeps the timing deterministic.
 *
 * All inputs/outputs must be float32. Dynamic dimensions are fixed to 1 (batch of one).
 * The ONNX `metadata_props` are exposed as ModelInfo::metadata.
 *
 * run() binds ONNX Runtime directly to the caller's tensor buffers (no copies). The bindings
 * are cached and only rebuilt when the caller passes different buffers.
 */
class OnnxInference : public Inference
{
public:
  OnnxInference();
  ~OnnxInference() override;

  /// @throws std::runtime_error when the model cannot be found/loaded or is not float32.
  void configure(const PluginContext & ctx) override;
  const ModelInfo & model() const override {return model_;}
  bool run(const TensorMap & inputs, TensorMap & outputs) override;

  /**
   * @brief Resolve a model path that may be a directory.
   * @throws std::runtime_error when nothing usable is found.
   */
  static std::string resolve_model_path(const std::string & path);

private:
  // ONNX Runtime stays out of this header, so that users of the class do not need it.
  struct Impl;
  std::unique_ptr<Impl> impl_;
  ModelInfo model_;
  rclcpp::Logger logger_{rclcpp::get_logger("OnnxInference")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__ONNX_INFERENCE_HPP_
