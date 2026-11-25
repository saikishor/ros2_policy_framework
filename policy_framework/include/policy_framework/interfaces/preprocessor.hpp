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


#ifndef POLICY_FRAMEWORK__INTERFACES__PREPROCESSOR_HPP_
#define POLICY_FRAMEWORK__INTERFACES__PREPROCESSOR_HPP_

#include "policy_framework/core/data_store.hpp"
#include "policy_framework/interfaces/plugin_context.hpp"
#include "policy_framework/interfaces/types.hpp"

/// @file preprocessor.hpp
/// @brief Base class of the Preprocessor plugins.

namespace policy_framework
{

/**
 * @brief Builds the model inputs from the state store (observation terms, history, ...).
 */
class Preprocessor
{
public:
  virtual ~Preprocessor() = default;
  /**
   * @brief Configure and register in @p state every entry this preprocessor reads.
   * @throws std::exception on any configuration error.
   */
  virtual void configure(const PluginContext & ctx, const ModelInfo & model, DataStore & state) = 0;
  /**
   * @brief Forget everything observed so far (e.g. the observation history). Called before
   *        the first activation and when the host (re)starts, including a safety reset after a
   *        latched stop (`~/reset_safety`), never in between.
   */
  virtual void clear() {}
  /**
   * @brief Called whenever the policy becomes active (after clear() if any), before the next
   *        compute(). Reset per-activation state here (e.g. a motion time step); state that
   *        should survive a switch away and back (e.g. history) belongs to clear().
   */
  virtual void reset() {}
  /**
   * @brief Fill the model inputs. Called from the control cycle.
   * @param state Latest values of all inputs.
   * @param last_outputs Model outputs of the previous step of this policy (zeros after
   *        clear()), e.g. to feed back the last action.
   * @param inputs Tensors to fill, shaped as ModelInfo::inputs.
   * @return false when the required data is not available yet (the cycle is skipped).
   */
  virtual bool compute(
    const DataStore & state, const TensorMap & last_outputs, TensorMap & inputs) = 0;
  /**
   * @brief Called from the control cycle on every step while another policy is active, instead
   *        of compute(). Keep time-dependent memory current here (e.g. the observation
   *        history), so that the policy does not resume from a stale past when switched back.
   * @param last_outputs Model outputs of the last step of this policy.
   */
  virtual void observe(const DataStore & state, const TensorMap & last_outputs)
  {
    (void)state;
    (void)last_outputs;
  }

  /**
   * @return One stamp per input of the last successful compute() (e.g. per observation term
   *         or per entry). The names and their order are fixed after configure(); only the
   *         stamps change. Empty when the preprocessor does not report stamps.
   */
  virtual const InputStamps & input_stamps() const
  {
    static const InputStamps none;
    return none;
  }
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__INTERFACES__PREPROCESSOR_HPP_
