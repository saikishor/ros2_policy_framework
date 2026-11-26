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

#include "policy_framework_mjlab/extra_inputs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

using policy_framework::DataStore;
using policy_framework::get_parameter;
using policy_framework::ModelInfo;
using policy_framework::PluginContext;
using policy_framework::TensorMap;

namespace policy_framework_mjlab
{

namespace
{

constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();

/// @throws std::runtime_error unless the model input @p input has exactly @p size values.
void require_size(const ModelInfo & model, const std::string & input, size_t size)
{
  const auto n = model.inputs.at(input).data.size();
  if (n != size) {
    throw std::runtime_error(
            "Model input '" + input + "' has " + std::to_string(n) + " values, expected " +
            std::to_string(size));
  }
}

/// @return The built-in ExtraInput of the model input @p name; null when there is none.
std::shared_ptr<ExtraInput> builtin(const std::string & name)
{
  if (name == "time_step") {return std::make_shared<TimeStepInput>();}
  if (name == "obstacle_height") {return std::make_shared<ParameterInput>();}
  return nullptr;
}

}  // namespace

void TimeStepInput::configure(
  const PluginContext &, const ModelInfo & model, const std::string & input, DataStore &)
{
  require_size(model, input, 1);
}

void TimeStepInput::compute(const DataStore &, const TensorMap &, float * out)
{
  out[0] = static_cast<float>(step_++);
}

void ParameterInput::configure(
  const PluginContext & ctx, const ModelInfo & model, const std::string & input, DataStore &)
{
  require_size(model, input, 1);
  const auto key = get_parameter<std::string>(ctx, "metadata_default", "reference_" + input);
  if (!key.empty()) {
    const auto it = model.metadata.find(key);
    if (it != model.metadata.end()) {default_ = std::stod(it->second);}
  }
  const auto full = ctx.name + ".value";
  value_param_ = get_parameter<double>(ctx, "value", kUnset);
  if (std::isnan(value_param_.load()) && std::isnan(default_)) {
    throw std::runtime_error(
            "Model input '" + input + "' has no default (metadata '" + key + "'): set " + full);
  }
  RCLCPP_INFO(
    ctx.node->get_logger(), "[INFO] [extra_inputs] [%s] param set: using %s = %f",
    ctx.name.c_str(), full.c_str(), value_param_.load());
  // Runtime changes, read at the next reset() (lock-free in the control cycle).
  param_callback_ = ctx.node->add_on_set_parameters_callback(
    [this, full, logger = ctx.node->get_logger(), name = ctx.name](
      const std::vector<rclcpp::Parameter> & ps) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : ps) {
        if (p.get_name() != full || p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          continue;
        }
        RCLCPP_INFO(
          logger, "[INFO] [extra_inputs] [%s] param callback: received %s = %f",
          name.c_str(), full.c_str(), p.as_double());
        if (std::isnan(p.as_double()) && std::isnan(default_)) {
          result.successful = false;
          result.reason = full + " cannot be unset: the model has no default";
          return result;
        }
        value_param_ = p.as_double();
      }
      return result;
    });
  reset();
  RCLCPP_INFO(
    ctx.node->get_logger(), "[%s] %s input: %.3f (model default %.3f)", ctx.name.c_str(),
    input.c_str(), static_cast<double>(value_), default_);
}

void ParameterInput::reset()
{
  const double v = value_param_.load();
  value_ = static_cast<float>(std::isnan(v) ? default_ : v);
}

void ParameterInput::compute(const DataStore &, const TensorMap &, float * out)
{
  out[0] = value_;
}

void ExtraInputs::configure(
  const PluginContext & ctx, const ModelInfo & model, DataStore & state,
  const std::string & observation_input, bool ignore_unknown)
{
  inputs_.clear();
  for (const auto & kv : model.inputs) {
    const auto & name = kv.first;
    if (name == observation_input) {continue;}
    const auto prefix = "extra_inputs." + name;
    // Not a generated parameter: the input names come from the model.
    const auto type = get_parameter<std::string>(ctx, prefix + ".type", "");
    std::shared_ptr<ExtraInput> filler;
    if (!type.empty()) {
      if (!loader_) {
        loader_ = std::make_unique<pluginlib::ClassLoader<ExtraInput>>(
          "policy_framework_mjlab", "policy_framework_mjlab::ExtraInput");
      }
      filler = loader_->createSharedInstance(type);
    } else {
      filler = builtin(name);
    }
    if (!filler) {
      if (ignore_unknown) {
        RCLCPP_WARN(
          ctx.node->get_logger(), "[%s] model input '%s' is not supported: left at zero",
          ctx.name.c_str(), name.c_str());
        continue;
      }
      throw std::runtime_error(
              "Model input '" + name + "' is not supported: set " + ctx.name + "." + prefix +
              ".type to an ExtraInput plugin, or " + ctx.name +
              ".unknown_extra_inputs to 'ignore' to leave it at zero");
    }
    filler->configure(
      {ctx.node, ctx.name + "." + prefix, ctx.kinematics, ctx.safety_entry}, model, name, state);
    inputs_.push_back({name, std::move(filler)});
  }
}

bool ExtraInputs::ready(const DataStore & state) const
{
  return std::all_of(
    inputs_.begin(), inputs_.end(), [&](const Entry & e) {return e.filler->ready(state);});
}

void ExtraInputs::reset()
{
  for (auto & e : inputs_) {e.filler->reset();}
}

void ExtraInputs::compute(
  const DataStore & state, const TensorMap & last_outputs, TensorMap & inputs)
{
  for (auto & e : inputs_) {
    e.filler->compute(state, last_outputs, inputs.at(e.input).data.data());
  }
}

}  // namespace policy_framework_mjlab

PLUGINLIB_EXPORT_CLASS(policy_framework_mjlab::TimeStepInput, policy_framework_mjlab::ExtraInput)
PLUGINLIB_EXPORT_CLASS(policy_framework_mjlab::ParameterInput, policy_framework_mjlab::ExtraInput)
