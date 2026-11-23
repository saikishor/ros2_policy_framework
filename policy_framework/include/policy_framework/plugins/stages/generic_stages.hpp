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

#ifndef POLICY_FRAMEWORK__PLUGINS__STAGES__GENERIC_STAGES_HPP_
#define POLICY_FRAMEWORK__PLUGINS__STAGES__GENERIC_STAGES_HPP_

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

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__PLUGINS__STAGES__GENERIC_STAGES_HPP_
