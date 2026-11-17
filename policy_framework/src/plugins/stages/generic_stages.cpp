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

#include "policy_framework/plugins/stages/generic_stages.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/concat_preprocessor_parameters.hpp"
#include "policy_framework/fixed_pose_postprocessor_parameters.hpp"
#include "policy_framework/split_postprocessor_parameters.hpp"

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
  const auto params = concat_preprocessor::ParamListener(ctx.node, ctx.name).get_params();
  tensor_ = params.input_tensor;
  const auto & entries = params.input_entries;
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
  const auto params = split_postprocessor::ParamListener(ctx.node, ctx.name).get_params();
  tensor_ = params.output_tensor;
  indices_ = register_entries(model.outputs, tensor_, params.output_entries, commands);
}

bool SplitPostprocessor::compute(const TensorMap & outputs, double stamp, DataStore & commands)
{
  const auto & data = outputs.at(tensor_).data;
  for (size_t i = 0; i < indices_.size(); ++i) {
    commands.set(indices_[i], data[i], stamp);
  }
  return true;
}

void FixedPosePostprocessor::configure(
  const PluginContext & ctx, const ModelInfo &, DataStore & commands)
{
  const auto params = fixed_pose_postprocessor::ParamListener(ctx.node, ctx.name).get_params();
  const auto & joints = params.joints;
  auto check_size = [&](const std::vector<double> & v, const std::string & what, bool required) {
      if ((required || !v.empty()) && v.size() != joints.size()) {
        throw std::runtime_error(
                ctx.name + ": " + what + " has " + std::to_string(v.size()) + " values for " +
                std::to_string(joints.size()) + " joints");
      }
    };
  check_size(params.positions, "positions", true);
  check_size(params.stiffness, "stiffness", false);
  check_size(params.damping, "damping", false);
  references_.clear();
  gains_.clear();
  for (size_t i = 0; i < joints.size(); ++i) {
    references_.push_back({commands.add(joints[i] + "/position"), params.positions[i]});
    references_.push_back({commands.add(joints[i] + "/velocity"), 0.0});
    if (!params.stiffness.empty()) {
      gains_.push_back({commands.add(joints[i] + "/stiffness"), params.stiffness[i]});
    }
    if (!params.damping.empty()) {
      gains_.push_back({commands.add(joints[i] + "/damping"), params.damping[i]});
    }
  }
}

void FixedPosePostprocessor::initial_commands(DataStore & commands)
{
  for (const auto & g : gains_) {commands.set(g.index, g.value, 0.0);}
}

bool FixedPosePostprocessor::compute(const TensorMap &, double stamp, DataStore & commands)
{
  for (const auto & r : references_) {commands.set(r.index, r.value, stamp);}
  for (const auto & g : gains_) {commands.set(g.index, g.value, stamp);}
  return true;
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::NullInference, policy_framework::Inference)
PLUGINLIB_EXPORT_CLASS(policy_framework::NullPreprocessor, policy_framework::Preprocessor)
PLUGINLIB_EXPORT_CLASS(policy_framework::FixedPosePostprocessor, policy_framework::Postprocessor)
PLUGINLIB_EXPORT_CLASS(policy_framework::ConcatPreprocessor, policy_framework::Preprocessor)
PLUGINLIB_EXPORT_CLASS(policy_framework::SplitPostprocessor, policy_framework::Postprocessor)
