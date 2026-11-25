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

#include "policy_framework/plugins/inference/onnx_inference.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <filesystem>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>
#include <fmt/format.h>

#include "policy_framework/onnx_inference_parameters.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace fs = std::filesystem;

namespace policy_framework
{
namespace
{

Ort::Env & ort_env()
{
  // One environment per process, shared by every policy.
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "policy_framework");
  return env;
}

Tensor make_tensor(const Ort::TypeInfo & info, const std::string & name)
{
  const auto shape_info = info.GetTensorTypeAndShapeInfo();
  if (shape_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
    throw std::runtime_error(fmt::format("ONNX tensor '{}' is not float32", name));
  }
  Tensor t;
  t.shape = shape_info.GetShape();
  for (auto & d : t.shape) {
    if (d < 0) {d = 1;}
  }
  const auto n = std::accumulate(
    t.shape.begin(), t.shape.end(), int64_t{1}, std::multiplies<int64_t>());
  t.data.assign(static_cast<size_t>(n), 0.0F);
  return t;
}

/// Re-create @p values when the buffers of @p tensors (in @p names order) changed.
void bind_tensors(
  const std::vector<std::string> & names, const TensorMap & tensors, const ModelInfo & model,
  const Ort::MemoryInfo & memory_info, std::vector<const float *> & bound,
  std::vector<Ort::Value> & values)
{
  bool same = bound.size() == names.size();
  for (size_t i = 0; same && i < names.size(); ++i) {
    same = tensors.at(names[i]).data.data() == bound[i];
  }
  if (same) {return;}
  // Only reached on the first run (or when the caller swaps buffers): allocates.
  bound.clear();
  values.clear();
  for (const auto & name : names) {
    const auto & t = tensors.at(name);
    const auto & spec = model.inputs.count(name) ? model.inputs.at(name) : model.outputs.at(name);
    if (t.data.size() != spec.data.size()) {
      throw std::runtime_error(fmt::format("Tensor '{}' has the wrong size", name));
    }
    // ORT never writes to input tensors; outputs are written in place (hence non-const data).
    values.push_back(
      Ort::Value::CreateTensor<float>(
        memory_info, const_cast<float *>(t.data.data()), t.data.size(),
        t.shape.data(), t.shape.size()));
    bound.push_back(t.data.data());
  }
}

}  // namespace

struct OnnxInference::Impl
{
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo memory_info{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
  std::vector<std::string> input_names, output_names;
  std::vector<const char *> input_cnames, output_cnames;
  std::vector<const float *> bound_inputs, bound_outputs;
  std::vector<Ort::Value> input_values, output_values;
};

OnnxInference::OnnxInference() = default;  // impl_ is created by configure()
OnnxInference::~OnnxInference() = default;

std::string OnnxInference::resolve_model_path(const std::string & path)
{
  if (path.empty()) {
    throw std::runtime_error("Empty model path");
  }
  if (fs::is_regular_file(path)) {
    return path;
  }
  if (!fs::is_directory(path)) {
    throw std::runtime_error(fmt::format("Model path does not exist: {}", path));
  }
  std::vector<std::string> candidates;
  for (const auto & entry : fs::directory_iterator(path)) {
    if (entry.is_regular_file() && entry.path().extension() == ".onnx") {
      candidates.push_back(entry.path().string());
    }
  }
  if (candidates.empty()) {
    throw std::runtime_error(fmt::format("No .onnx file in directory: {}", path));
  }
  return *std::min_element(candidates.begin(), candidates.end());
}

void OnnxInference::configure(const PluginContext & ctx)
{
  logger_ = ctx.node->get_logger().get_child(ctx.name);
  const auto params = onnx_inference::ParamListener(ctx.node, ctx.name).get_params();
  const auto model_path = resolve_model_path(params.model_path);
  const auto num_threads = params.num_threads;

  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(static_cast<int>(num_threads));
  options.SetInterOpNumThreads(1);
  impl_ = std::make_unique<Impl>();
  auto & s = *impl_;
  s.session = std::make_unique<Ort::Session>(ort_env(), model_path.c_str(), options);

  model_ = ModelInfo{};
  Ort::AllocatorWithDefaultOptions allocator;
  auto metadata = s.session->GetModelMetadata();
  for (const auto & key : metadata.GetCustomMetadataMapKeysAllocated(allocator)) {
    auto value = metadata.LookupCustomMetadataMapAllocated(key.get(), allocator);
    model_.metadata[key.get()] = value ? value.get() : "";
  }

  for (size_t i = 0; i < s.session->GetInputCount(); ++i) {
    s.input_names.push_back(s.session->GetInputNameAllocated(i, allocator).get());
    model_.inputs[s.input_names.back()] =
      make_tensor(s.session->GetInputTypeInfo(i), s.input_names.back());
  }
  for (size_t i = 0; i < s.session->GetOutputCount(); ++i) {
    s.output_names.push_back(s.session->GetOutputNameAllocated(i, allocator).get());
    model_.outputs[s.output_names.back()] =
      make_tensor(s.session->GetOutputTypeInfo(i), s.output_names.back());
  }
  for (const auto & n : s.input_names) {s.input_cnames.push_back(n.c_str());}
  for (const auto & n : s.output_names) {s.output_cnames.push_back(n.c_str());}

  RCLCPP_INFO(logger_, "Loaded ONNX model %s", model_path.c_str());
  for (const auto & [name, t] : model_.inputs) {
    RCLCPP_INFO(logger_, "  input  '%s': %zu values", name.c_str(), t.data.size());
  }
  for (const auto & [name, t] : model_.outputs) {
    RCLCPP_INFO(logger_, "  output '%s': %zu values", name.c_str(), t.data.size());
  }
}

bool OnnxInference::run(const TensorMap & inputs, TensorMap & outputs)
{
  auto & s = *impl_;
  try {
    bind_tensors(s.input_names, inputs, model_, s.memory_info, s.bound_inputs, s.input_values);
    bind_tensors(s.output_names, outputs, model_, s.memory_info, s.bound_outputs, s.output_values);
    s.session->Run(
      Ort::RunOptions{nullptr}, s.input_cnames.data(), s.input_values.data(),
      s.input_values.size(), s.output_cnames.data(), s.output_values.data(),
      s.output_values.size());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      logger_, steady_clock_, 1000, "ONNX inference failed: %s", e.what());
    return false;
  }
  return true;
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::OnnxInference, policy_framework::Inference)
