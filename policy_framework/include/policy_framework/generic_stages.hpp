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
 */
class ConcatPreprocessor : public Preprocessor
{
public:
  /// @throws std::runtime_error on unknown tensor or size mismatch.
  void configure(const PluginContext & ctx, const ModelInfo & model, DataStore & state) override;
  bool compute(
    const DataStore & state, const TensorMap & last_outputs, TensorMap & inputs) override;

private:
  std::string tensor_;
  std::vector<size_t> indices_;
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
  bool compute(const TensorMap & outputs, DataStore & commands) override;

private:
  std::string tensor_;
  std::vector<size_t> indices_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__GENERIC_STAGES_HPP_
