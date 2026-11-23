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


#ifndef POLICY_FRAMEWORK__INTERFACES__PLUGIN_CONTEXT_HPP_
#define POLICY_FRAMEWORK__INTERFACES__PLUGIN_CONTEXT_HPP_

#include <memory>
#include <optional>
#include <string>

#include "policy_framework/core/data_store.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

/// @file plugin_context.hpp
/// @brief What a plugin is configured with (PluginContext), its parameters and the CommandGate.

namespace policy_framework
{

class Kinematics;

/// What a plugin needs to configure itself.
struct PluginContext
{
  /// Node of the runner (standalone node or controller), used for parameters, topics, timers.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  /// Name of the policy, source, executor or term; plugin parameters live under `<name>.`.
  std::string name;
  /// Whole-body kinematics (null unless `kinematics.enabled`) of the state the policy step
  /// reads, updated before every step; given to the policy stages and their observation terms
  /// only.
  std::shared_ptr<Kinematics> kinematics = nullptr;
  /// State entry that is 0 while user commands must be ignored (see CommandGate); empty: none.
  std::string safety_entry = "";
};

/**
 * @brief State entry that the host sets above 0 while a fault with the NEUTRAL_COMMANDS
 *        reaction is active (see CommandGate). Registered and written by the runner itself, so
 *        neither an input source nor a state interface provides it.
 */
inline constexpr char kNeutralCommandsEntry[] = "policy_framework/neutral_commands";

/**
 * @brief Whether user commands (a velocity command, a motion reference...) may be used now.
 *
 * With the host parameter `safety_entry` set (e.g. `rl_safety_controller/safe`, a state
 * interface in the controller), commands are only accepted while that state entry was received
 * and is greater than 0. Independently of it, commands are refused while the host sets
 * kNeutralCommandsEntry above 0 (a NEUTRAL_COMMANDS fault reaction), and accepted again as soon
 * as it clears it. Plugins that feed user commands to a policy check open() and use their
 * neutral command (e.g. zero velocity) while it is false; the policy itself keeps running.
 */
class CommandGate
{
public:
  CommandGate() = default;
  explicit CommandGate(const PluginContext & ctx)
  : name_(ctx.safety_entry) {}

  /// Real-time safe after the first call (which looks the entries up in @p state).
  bool open(const DataStore & state)
  {
    if (!resolved_) {
      if (!name_.empty()) {index_ = state.find(name_);}
      neutral_index_ = state.find(kNeutralCommandsEntry);
      resolved_ = true;
    }
    if (neutral_index_ && state.received(*neutral_index_) && state.get(*neutral_index_) > 0.0) {
      return false;
    }
    if (name_.empty()) {return true;}
    return index_ && state.received(*index_) && state.get(*index_) > 0.0;
  }

private:
  std::string name_;
  std::optional<size_t> index_;
  std::optional<size_t> neutral_index_;
  bool resolved_ = false;
};

/**
 * @brief Declare (when not declared yet) and return the plugin parameter `<ctx.name>.<param>`.
 *
 * Several policies can share a plugin type, and the node and the controller both re-run
 * configuration, hence the "declare once" semantics.
 */
template<typename T>
T get_parameter(const PluginContext & ctx, const std::string & param, const T & default_value)
{
  const auto full = ctx.name + "." + param;
  if (!ctx.node->has_parameter(full)) {ctx.node->declare_parameter<T>(full, default_value);}
  return ctx.node->get_parameter(full).template get_value<T>();
}

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__INTERFACES__PLUGIN_CONTEXT_HPP_
