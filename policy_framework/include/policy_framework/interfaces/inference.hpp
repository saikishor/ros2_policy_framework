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


#ifndef POLICY_FRAMEWORK__INTERFACES__INFERENCE_HPP_
#define POLICY_FRAMEWORK__INTERFACES__INFERENCE_HPP_

#include "policy_framework/interfaces/plugin_context.hpp"
#include "policy_framework/interfaces/types.hpp"

/// @file inference.hpp
/// @brief Base class of the Inference plugins.

namespace policy_framework
{

/**
 * @brief Runs the model. The only framework-specific stage (ONNX, TensorRT, ...).
 */
class Inference
{
public:
  virtual ~Inference() = default;
  /**
   * @brief Load the model.
   * @throws std::exception on any configuration or loading error.
   */
  virtual void configure(const PluginContext & ctx) = 0;
  /// @return Description of the loaded model. Valid after configure().
  virtual const ModelInfo & model() const = 0;
  /**
   * @brief Run one inference step. Called from the control cycle.
   * @param inputs Tensors with the shapes of model().inputs.
   * @param outputs Tensors with the shapes of model().outputs; overwritten in place.
   * @return false on failure.
   */
  virtual bool run(const TensorMap & inputs, TensorMap & outputs) = 0;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__INTERFACES__INFERENCE_HPP_
