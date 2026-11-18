// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__GENERIC_STAGES_HPP_
#define POLICY_FRAMEWORK__GENERIC_STAGES_HPP_

#include <string>
#include <vector>

#include "policy_framework/plugin_interfaces.hpp"

namespace policy_framework
{

/**
 * @brief Preprocessor that concatenates state entries into one model input, as-is.
 *
 * For simple models whose observation needs no history or transformation. Model inputs that
 * are not `input_tensor` are left at zero.
 *
 * Parameters (under `<policy>.`):
 * - `input_tensor` (string, default `obs`)
 * - `input_entries` (string array): state entries, in order; must match the tensor size.
 *
 * input_stamps(): one per entry, the stamp of the entry in the state store.
 */
class ConcatPreprocessor : public Preprocessor
{
public:
  /// @throws std::runtime_error on unknown tensor or size mismatch.
  void configure(const PluginContext & ctx, const ModelInfo & model, DataStore & state) override;
  bool compute(
    const DataStore & state, const TensorMap & last_outputs, TensorMap & inputs) override;
  const InputStamps & input_stamps() const override {return stamps_;}

private:
  std::string tensor_;
  std::vector<size_t> indices_;
  InputStamps stamps_;
};

/**
 * @brief Postprocessor that writes each value of one model output to a command entry, as-is.
 *
 * Parameters (under `<policy>.`):
 * - `output_tensor` (string, default `actions`)
 * - `output_entries` (string array): command entries, in order; must match the tensor size.
 */
class SplitPostprocessor : public Postprocessor
{
public:
  /// @throws std::runtime_error on unknown tensor or size mismatch.
  void configure(const PluginContext & ctx, const ModelInfo & model, DataStore & commands) override;
  bool compute(const TensorMap & outputs, double stamp, DataStore & commands) override;

private:
  std::string tensor_;
  std::vector<size_t> indices_;
};

/**
 * @brief Inference of a policy without a model: no tensors, always succeeds.
 *
 * With NullPreprocessor and a postprocessor that needs no model output (e.g.
 * FixedPosePostprocessor) it makes a model-free policy, switched like any other.
 */
class NullInference : public Inference
{
public:
  void configure(const PluginContext &) override {}
  const ModelInfo & model() const override {return model_;}
  bool run(const TensorMap &, TensorMap &) override {return true;}

private:
  ModelInfo model_;
};

/// Preprocessor of a policy without inputs: reads nothing and is always ready.
class NullPreprocessor : public Preprocessor
{
public:
  void configure(const PluginContext &, const ModelInfo &, DataStore &) override {}
  bool compute(const DataStore &, const TensorMap &, TensorMap &) override {return true;}
};

/**
 * @brief Postprocessor that commands a fixed joint pose, whatever the model answers.
 *
 * For a standby or stand-up policy: with NullInference and NullPreprocessor, and the policy's
 * `transition_time`, switching to it moves the robot to the pose in that time.
 *
 * Parameters (under `<policy>.`):
 * - `joints` (string array) and `positions` (double array, one per joint): written as
 *   `<joint>/position`, with `<joint>/velocity` = 0.
 * - `stiffness`, `damping` (double arrays, one per joint, default empty): written as
 *   `<joint>/stiffness` and `<joint>/damping` (for PdEffortExecutor), also as initial commands.
 */
class FixedPosePostprocessor : public Postprocessor
{
public:
  /// @throws std::runtime_error when an array does not have one value per joint.
  void configure(const PluginContext & ctx, const ModelInfo & model, DataStore & commands) override;
  void initial_commands(DataStore & commands) override;
  bool compute(const TensorMap & outputs, double stamp, DataStore & commands) override;

private:
  struct Value
  {
    size_t index;
    double value;
  };
  std::vector<Value> references_;  ///< Positions and zero velocities.
  std::vector<Value> gains_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__GENERIC_STAGES_HPP_
