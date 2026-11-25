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


#ifndef POLICY_FRAMEWORK__INTERFACES__TYPES_HPP_
#define POLICY_FRAMEWORK__INTERFACES__TYPES_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

/// @file types.hpp
/// @brief Data types shared by the plugin interfaces: tensors, model description, fault reactions, input stamps.

namespace policy_framework
{

/// Dense float32 tensor in row-major order.
struct Tensor
{
  std::vector<int64_t> shape;
  std::vector<float> data;
};

/// Tensors keyed by model input/output name.
using TensorMap = std::map<std::string, Tensor>;

/// Static description of a loaded model, produced by the Inference stage.
struct ModelInfo
{
  /// Custom model metadata (e.g. ONNX `metadata_props`).
  std::map<std::string, std::string> metadata;
  /// One zero-filled tensor per model input, with its (static) shape.
  TensorMap inputs;
  /// One zero-filled tensor per model output, with its (static) shape.
  TensorMap outputs;
};

/**
 * @brief What a host does about a fault (a failed safety check, a failed policy step...).
 *
 * Ordered from the most control kept to the least: a host only ever escalates to a later
 * reaction. NEUTRAL_COMMANDS lifts as soon as no fault asks for it any more; every other
 * reaction is latched until the host is reactivated or its safety is reset (`~/reset_safety`,
 * PolicyRunner::apply_safety_reset()). PolicyRunner::update_safety() applies
 * them; policy_node stops its commands on a latched one, PolicyController makes its executors
 * apply it (InterfaceExecutor::fault()) or returns an error.
 */
enum class FaultReaction
{
  /// The policy keeps running, with neutral user commands (e.g. zero velocity, see
  /// CommandGate): the executors keep following it.
  NEUTRAL_COMMANDS,
  HOLD,   ///< Keep the pose of the fault, e.g. a PD law around it.
  DAMP,   ///< Let the robot down gently, e.g. a pure joint damping.
  SAFE,   ///< The safe values, e.g. zero effort.
  ERROR   ///< Stop: the host returns an error and is deactivated (fallback controllers take over).
};

/**
 * @return The reaction named @p name in a configuration (`neutral_commands`, `hold`, `damp`,
 *         `safe`, `error`), or nothing for any other name.
 */
inline std::optional<FaultReaction> fault_reaction_from_string(const std::string & name)
{
  if (name == "neutral_commands") {return FaultReaction::NEUTRAL_COMMANDS;}
  if (name == "hold") {return FaultReaction::HOLD;}
  if (name == "damp") {return FaultReaction::DAMP;}
  if (name == "safe") {return FaultReaction::SAFE;}
  if (name == "error") {return FaultReaction::ERROR;}
  return std::nullopt;
}

/// @return The configuration name of @p reaction (see fault_reaction_from_string()).
inline const char * to_string(FaultReaction reaction)
{
  switch (reaction) {
    case FaultReaction::NEUTRAL_COMMANDS: return "neutral_commands";
    case FaultReaction::HOLD: return "hold";
    case FaultReaction::DAMP: return "damp";
    case FaultReaction::SAFE: return "safe";
    case FaultReaction::ERROR: return "error";
  }
  return "unknown";
}

/**
 * @brief When the data of one input of a policy step was produced.
 *
 * Stamps are seconds on the node clock: the message header stamp for topic data (the current
 * time when the header is empty or the message has none), the cycle time for state
 * interfaces (ros2_control interfaces carry no stamp). 0 means unknown (e.g. never received).
 * They let a host measure how old each input is and the latency down to the commands.
 */
struct InputStamp
{
  std::string name;
  double stamp = 0.0;
  /// The input is fed back from the policy's previous outputs (e.g. the last action) and has
  /// the stamp of the previous observation; it does not count for the observation stamp.
  bool from_previous_outputs = false;
  /// The input is a user command (a velocity command, a motion reference): held until its own
  /// timeout, so it is old by design; it does not count for the observation stamp.
  bool user_command = false;
  /// When the latest message of the input arrived (seconds, node clock), for an input with its
  /// own subscription (a TopicTerm); 0 otherwise or before any.
  double receive_time = 0.0;
  /// Largest age of receive_time (seconds) before the host faults with max_age_on_fault; 0: off.
  double max_age = 0.0;
  FaultReaction max_age_on_fault = FaultReaction::ERROR;
};
using InputStamps = std::vector<InputStamp>;

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__INTERFACES__TYPES_HPP_
