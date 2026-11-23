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


#ifndef POLICY_FRAMEWORK__INTERFACES__POSTPROCESSOR_HPP_
#define POLICY_FRAMEWORK__INTERFACES__POSTPROCESSOR_HPP_

#include "policy_framework/core/data_store.hpp"
#include "policy_framework/interfaces/plugin_context.hpp"
#include "policy_framework/interfaces/types.hpp"

/// @file postprocessor.hpp
/// @brief Base class of the Postprocessor plugins.

namespace policy_framework
{

/**
 * @brief Converts the model outputs into named commands (scaling, offsets, limits, ...).
 */
class Postprocessor
{
public:
  virtual ~Postprocessor() = default;
  /**
   * @brief Configure and register in @p commands every entry this postprocessor writes.
   * @throws std::exception on any configuration error.
   */
  virtual void configure(
    const PluginContext & ctx, const ModelInfo & model, DataStore & commands) = 0;
  /// Called from the control cycle whenever the policy becomes active. Must be real-time safe.
  virtual void reset() {}

  /**
   * @brief Write the commands that are already known before the first step, e.g. the policy's
   *        gains; not references (the executors hold the robot until the first step, see
   *        InterfaceExecutor::hold()). Called for the default policy when the host starts,
   *        outside the control cycle.
   */
  virtual void initial_commands(DataStore & commands) {(void)commands;}
  /**
   * @brief Write the commands. Called from the control cycle.
   * @param stamp Stamp of the observation the outputs were computed from (oldest input, see
   *        PolicyPipeline::observation_stamp()); write it as the stamp of every command.
   * @return false on failure.
   */
  virtual bool compute(const TensorMap & outputs, double stamp, DataStore & commands) = 0;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__INTERFACES__POSTPROCESSOR_HPP_
