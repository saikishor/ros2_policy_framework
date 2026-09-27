// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/onnx_inference.hpp"

#include <algorithm>
#include <filesystem>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

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
    throw std::runtime_error("ONNX tensor '" + name + "' is not float32");
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

}  // namespace

std::string OnnxInference::resolve_model_path(const std::string & path)
{
  if (path.empty()) {
    throw std::runtime_error("Empty model path");
  }
  if (fs::is_regular_file(path)) {
    return path;
  }
  if (!fs::is_directory(path)) {
    throw std::runtime_error("Model path does not exist: " + path);
  }
  std::vector<std::string> candidates;
  for (const auto & entry : fs::directory_iterator(path)) {
    if (entry.is_regular_file() && entry.path().extension() == ".onnx") {
      candidates.push_back(entry.path().string());
    }
  }
  if (candidates.empty()) {
    throw std::runtime_error("No .onnx file in directory: " + path);
  }
  return *std::min_element(candidates.begin(), candidates.end());
}

void OnnxInference::configure(const PluginContext & ctx)
{
  logger_ = ctx.node->get_logger().get_child(ctx.name);
  const auto model_path = resolve_model_path(get_parameter<std::string>(ctx, "model_path", ""));
  const auto num_threads = get_parameter<int64_t>(ctx, "num_threads", 1);

  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(static_cast<int>(num_threads));
  options.SetInterOpNumThreads(1);
  session_ = std::make_unique<Ort::Session>(ort_env(), model_path.c_str(), options);

  model_ = ModelInfo{};
  Ort::AllocatorWithDefaultOptions allocator;
  auto metadata = session_->GetModelMetadata();
  for (const auto & key : metadata.GetCustomMetadataMapKeysAllocated(allocator)) {
    auto value = metadata.LookupCustomMetadataMapAllocated(key.get(), allocator);
    model_.metadata[key.get()] = value ? value.get() : "";
  }

  input_names_.clear();
  output_names_.clear();
  for (size_t i = 0; i < session_->GetInputCount(); ++i) {
    input_names_.push_back(session_->GetInputNameAllocated(i, allocator).get());
    model_.inputs[input_names_.back()] =
      make_tensor(session_->GetInputTypeInfo(i), input_names_.back());
  }
  for (size_t i = 0; i < session_->GetOutputCount(); ++i) {
    output_names_.push_back(session_->GetOutputNameAllocated(i, allocator).get());
    model_.outputs[output_names_.back()] =
      make_tensor(session_->GetOutputTypeInfo(i), output_names_.back());
  }
  input_cnames_.clear();
  output_cnames_.clear();
  for (const auto & n : input_names_) {input_cnames_.push_back(n.c_str());}
  for (const auto & n : output_names_) {output_cnames_.push_back(n.c_str());}
  bound_inputs_.clear();
  bound_outputs_.clear();

  RCLCPP_INFO(logger_, "Loaded ONNX model %s", model_path.c_str());
  for (const auto & [name, t] : model_.inputs) {
    RCLCPP_INFO(logger_, "  input  '%s': %zu values", name.c_str(), t.data.size());
  }
  for (const auto & [name, t] : model_.outputs) {
    RCLCPP_INFO(logger_, "  output '%s': %zu values", name.c_str(), t.data.size());
  }
}

void OnnxInference::bind(
  const std::vector<std::string> & names, const TensorMap & tensors,
  std::vector<const float *> & bound, std::vector<Ort::Value> & values)
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
    const auto & spec = model_.inputs.count(name) ? model_.inputs.at(name) :
      model_.outputs.at(name);
    if (t.data.size() != spec.data.size()) {
      throw std::runtime_error("Tensor '" + name + "' has the wrong size");
    }
    // ORT never writes to input tensors; outputs are written in place (hence non-const data).
    values.push_back(
      Ort::Value::CreateTensor<float>(
        memory_info_, const_cast<float *>(t.data.data()), t.data.size(),
        t.shape.data(), t.shape.size()));
    bound.push_back(t.data.data());
  }
}

bool OnnxInference::run(const TensorMap & inputs, TensorMap & outputs)
{
  try {
    bind(input_names_, inputs, bound_inputs_, input_values_);
    bind(output_names_, outputs, bound_outputs_, output_values_);
    session_->Run(
      Ort::RunOptions{nullptr}, input_cnames_.data(), input_values_.data(), input_values_.size(),
      output_cnames_.data(), output_values_.data(), output_values_.size());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      logger_, steady_clock_, 1000, "ONNX inference failed: %s", e.what());
    return false;
  }
  return true;
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::OnnxInference, policy_framework::Inference)
