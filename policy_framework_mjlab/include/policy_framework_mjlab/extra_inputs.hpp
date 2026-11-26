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

#ifndef POLICY_FRAMEWORK_MJLAB__EXTRA_INPUTS_HPP_
#define POLICY_FRAMEWORK_MJLAB__EXTRA_INPUTS_HPP_

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "policy_framework/plugin_interfaces.hpp"

/// @file extra_inputs.hpp
/// @brief Model inputs other than the observation (time_step, obstacle_height...), as plugins.

namespace policy_framework_mjlab
{

/**
 * @brief Fills one model input that is not the observation, e.g. the `time_step` of an
 *        imitation policy or the `obstacle_height` of a box-drop policy.
 *
 * Unlike an ObservationTerm it has no history: it writes the whole input tensor on every
 * compute(). Plugins of this class are listed under the base class package
 * `policy_framework_mjlab`.
 */
// ponytail: nothing here is mjlab-specific (ExtraInput, TimeStepInput, ParameterInput and
// ExtraInputs only use policy_framework types). If another preprocessor needs model inputs
// besides its observation, move them to policy_framework/interfaces next to ObservationTerm,
// with the plugin base class package changed to `policy_framework` (ClassLoader in
// ExtraInputs::configure(), pluginlib_export_plugin_description_file in CMakeLists.txt) and
// the plugin names to `policy_framework/...`. That breaks configurations that set
// `extra_inputs.<input>.type`. The `reference_<input>` metadata default of ParameterInput is a
// pal_mjlab convention: keep it configurable (`metadata_default`) if moved.
class ExtraInput
{
public:
  virtual ~ExtraInput() = default;
  /**
   * @brief Configure; register in @p state every entry read.
   * @param ctx Parameters of the input live under `<ctx.name>.`
   *        (`<policy>.extra_inputs.<input>.`).
   * @param input Name of the model input to fill (a key of ModelInfo::inputs).
   * @throws std::exception on any configuration error (e.g. an unexpected tensor size).
   */
  virtual void configure(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
    const std::string & input, policy_framework::DataStore & state) = 0;
  /// @return false while the data is not available yet (the cycle is skipped). Control cycle.
  virtual bool ready(const policy_framework::DataStore &) {return true;}
  /// Called whenever the policy becomes active. Real-time safe.
  virtual void reset() {}
  /**
   * @brief Write the input. Called from the control cycle, only when ready().
   * @param out The input tensor's data (its size was checked in configure()).
   */
  virtual void compute(
    const policy_framework::DataStore & state, const policy_framework::TensorMap & last_outputs,
    float * out) = 0;
};

/**
 * @brief `policy_framework_mjlab/TimeStepInput`: the number of policy steps since reset()
 *        (0 on the first step after activation). The input must have a single value.
 */
class TimeStepInput : public ExtraInput
{
public:
  void configure(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
    const std::string & input, policy_framework::DataStore & state) override;
  void reset() override {step_ = 0;}
  void compute(
    const policy_framework::DataStore &, const policy_framework::TensorMap &,
    float * out) override;

private:
  int64_t step_ = 0;
};

/**
 * @brief `policy_framework_mjlab/ParameterInput`: a scalar from a parameter, changeable at
 *        runtime, latched at reset() so that a running motion keeps its value.
 *
 * Parameters (under `<policy>.extra_inputs.<input>.`):
 * - `value` (double, default NaN: unset): the value; unset (or set to `.nan`) uses the metadata
 *   default. Change it with `ros2 param set <node> <policy>.extra_inputs.<input>.value <v>`,
 *   used from the next activation.
 * - `metadata_default` (string, default `reference_<input>`): the model metadata key holding
 *   the default value; empty for none.
 *
 * @throws std::runtime_error in configure() when neither `value` nor the metadata default is
 *         set, or the input has more than one value.
 */
class ParameterInput : public ExtraInput
{
public:
  void configure(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
    const std::string & input, policy_framework::DataStore & state) override;
  void reset() override;
  void compute(
    const policy_framework::DataStore &, const policy_framework::TensorMap &,
    float * out) override;

private:
  double default_ = std::numeric_limits<double>::quiet_NaN();  ///< From the metadata, NaN: none.
  std::atomic<double> value_param_{0.0};  ///< Set by the parameter callback, NaN: unset.
  float value_ = 0.0F;  ///< Latched at reset().
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  rclcpp::Logger logger_ = rclcpp::get_logger("extra_inputs");  ///< Set in configure().
  std::string name_;  ///< The plugin's parameter prefix, for the logs.
};

/**
 * @brief Every model input other than those a preprocessor fills itself (the observation),
 *        each filled by an ExtraInput.
 *
 * The ExtraInput of input `<input>`: the plugin `<policy>.extra_inputs.<input>.type` when set,
 * otherwise the built-in one of that name:
 * - `time_step`: TimeStepInput
 * - `obstacle_height`: ParameterInput (default: the `reference_obstacle_height` metadata)
 *
 * An input with neither is an error unless @p ignore_unknown (then it stays at zero).
 */
class ExtraInputs
{
public:
  /**
   * @param observation_input The model input filled by the caller, skipped here.
   * @param ignore_unknown Leave an input with no ExtraInput at zero instead of throwing.
   * @throws std::exception on an unsupported input or a configuration error of an input.
   */
  void configure(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
    policy_framework::DataStore & state, const std::string & observation_input,
    bool ignore_unknown);
  /// @return true when every input is ready(). Control cycle.
  bool ready(const policy_framework::DataStore & state) const;
  /// Called whenever the policy becomes active. Real-time safe.
  void reset();
  /// Fill every input of @p inputs this handles. Control cycle, only when ready().
  void compute(
    const policy_framework::DataStore & state, const policy_framework::TensorMap & last_outputs,
    policy_framework::TensorMap & inputs);

private:
  struct Entry
  {
    std::string input;
    std::shared_ptr<ExtraInput> filler;
  };

  // Declared before inputs_ so that it outlives the plugin instances.
  std::unique_ptr<pluginlib::ClassLoader<ExtraInput>> loader_;
  std::vector<Entry> inputs_;
};

}  // namespace policy_framework_mjlab

#endif  // POLICY_FRAMEWORK_MJLAB__EXTRA_INPUTS_HPP_
