// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__ONNX_INFERENCE_HPP_
#define POLICY_FRAMEWORK__ONNX_INFERENCE_HPP_

#include <onnxruntime_cxx_api.h>

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
  /// Re-create @p values when the buffers of @p tensors (in @p names order) changed.
  void bind(
    const std::vector<std::string> & names, const TensorMap & tensors,
    std::vector<const float *> & bound, std::vector<Ort::Value> & values);

  ModelInfo model_;
  std::unique_ptr<Ort::Session> session_;
  Ort::MemoryInfo memory_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
  std::vector<std::string> input_names_, output_names_;
  std::vector<const char *> input_cnames_, output_cnames_;
  std::vector<const float *> bound_inputs_, bound_outputs_;
  std::vector<Ort::Value> input_values_, output_values_;
  rclcpp::Logger logger_{rclcpp::get_logger("OnnxInference")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__ONNX_INFERENCE_HPP_
