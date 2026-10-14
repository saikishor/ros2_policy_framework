// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/generic_stages.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace policy_framework
{
namespace
{

/// Register @p entries in @p store, checking they match the size of @p tensor in @p tensors.
std::vector<size_t> register_entries(
  const TensorMap & tensors, const std::string & tensor, const std::vector<std::string> & entries,
  DataStore & store)
{
  const auto it = tensors.find(tensor);
  if (it == tensors.end()) {
    throw std::runtime_error("Model has no tensor '" + tensor + "'");
  }
  if (it->second.data.size() != entries.size()) {
    throw std::runtime_error(
            "Tensor '" + tensor + "' has " + std::to_string(it->second.data.size()) +
            " values but " + std::to_string(entries.size()) + " entries are configured");
  }
  std::vector<size_t> indices;
  for (const auto & e : entries) {indices.push_back(store.add(e));}
  return indices;
}

}  // namespace

void ConcatPreprocessor::configure(
  const PluginContext & ctx, const ModelInfo & model, DataStore & state)
{
  tensor_ = get_parameter<std::string>(ctx, "input_tensor", "obs");
  const auto entries = get_parameter<std::vector<std::string>>(ctx, "input_entries", {});
  indices_ = register_entries(model.inputs, tensor_, entries, state);
  stamps_.clear();
  for (const auto & e : entries) {stamps_.push_back({e, 0.0});}
}

bool ConcatPreprocessor::compute(const DataStore & state, const TensorMap &, TensorMap & inputs)
{
  if (!state.all_received(indices_)) {return false;}
  auto & data = inputs.at(tensor_).data;
  for (size_t i = 0; i < indices_.size(); ++i) {
    data[i] = static_cast<float>(state.get(indices_[i]));
    stamps_[i].stamp = state.stamp(indices_[i]);
  }
  return true;
}

void SplitPostprocessor::configure(
  const PluginContext & ctx, const ModelInfo & model, DataStore & commands)
{
  tensor_ = get_parameter<std::string>(ctx, "output_tensor", "actions");
  indices_ = register_entries(
    model.outputs, tensor_,
    get_parameter<std::vector<std::string>>(ctx, "output_entries", {}), commands);
}

bool SplitPostprocessor::compute(const TensorMap & outputs, DataStore & commands)
{
  const auto & data = outputs.at(tensor_).data;
  for (size_t i = 0; i < indices_.size(); ++i) {
    commands.set(indices_[i], data[i], 0.0);
  }
  return true;
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::ConcatPreprocessor, policy_framework::Preprocessor)
PLUGINLIB_EXPORT_CLASS(policy_framework::SplitPostprocessor, policy_framework::Postprocessor)
