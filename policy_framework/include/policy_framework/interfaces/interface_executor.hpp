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


#ifndef POLICY_FRAMEWORK__INTERFACES__INTERFACE_EXECUTOR_HPP_
#define POLICY_FRAMEWORK__INTERFACES__INTERFACE_EXECUTOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "policy_framework/core/data_store.hpp"
#include "policy_framework/interfaces/plugin_context.hpp"
#include "policy_framework/interfaces/types.hpp"

/// @file interface_executor.hpp
/// @brief Base class of the InterfaceExecutor plugins (ros2_control command interfaces).

namespace policy_framework
{

/**
 * @brief Turns the commands into ros2_control command interface values, inside
 *        PolicyController (the controller's counterpart of the topic Executor, which only
 *        runs in the standalone node).
 *
 * It may compute (e.g. a PD law into efforts) and runs on every controller update, also
 * between policy steps when the policy runs at a lower rate, with the state interfaces read
 * in that update. It does not touch loaned interfaces: it fills values that the controller
 * writes, so it can be tested without a controller manager.
 */
class InterfaceExecutor
{
public:
  virtual ~InterfaceExecutor() = default;
  /**
   * @brief Configure. @p commands already holds every entry the policies may write.
   * @param state Register every state entry read in write(); the controller claims them as
   *        state interfaces, read on every update.
   * @throws std::exception on any configuration error.
   */
  virtual void configure(const PluginContext & ctx, const DataStore & commands, DataStore & state) = 0;
  /// @return Full names of the command interfaces to claim. Valid after configure().
  virtual const std::vector<std::string> & command_interfaces() const = 0;
  /**
   * @return State entries read in write() that must be fresh on every update (e.g. joint
   *         feedback of a PD law); configuration fails if an input source provides one.
   */
  virtual std::vector<std::string> state_entries() const {return {};}
  /**
   * @brief Compute the interface values. Called from the control loop on every update, once
   *        the active policy has produced commands since the controller was activated (until
   *        then: hold()).
   * @param values One per command_interfaces(); NaN leaves that interface unchanged.
   */
  virtual void write(const DataStore & commands, const DataStore & state, std::vector<double> & values) = 0;
  /**
   * @brief Compute the interface values that keep the robot where it is. Called from the
   *        control loop on every update from the activation of the controller until the active
   *        policy produced its first commands (e.g. while it waits for its inputs, or for its
   *        first inference), instead of write().
   * @param commands Only holds the policy's Postprocessor::initial_commands().
   * @param start true on the first call after an activation.
   * @param values One per command_interfaces(); NaN leaves that interface unchanged. Default:
   *        all NaN.
   */
  virtual void hold(
    const DataStore & commands, const DataStore & state, std::vector<double> & values, bool start)
  {
    (void)commands;
    (void)state;
    (void)start;
    values.assign(values.size(), std::numeric_limits<double>::quiet_NaN());
  }

  /**
   * @brief Values written when the controller stops commanding: on deactivation and when the
   *        policy fails. Default: NaN (leave every interface unchanged).
   */
  virtual void safe_values(std::vector<double> & values)
  {
    values.assign(values.size(), std::numeric_limits<double>::quiet_NaN());
  }

  /**
   * @return true when fault() can apply @p reaction with this configuration (e.g. DAMP needs a
   *         damping). Meant for hosts to refuse, while configuring, a reaction that their
   *         configuration uses but an executor cannot apply. Default: NEUTRAL_COMMANDS (the
   *         executor keeps following the policy), SAFE and ERROR.
   */
  virtual bool supports_fault_reaction(FaultReaction reaction) const
  {
    return reaction == FaultReaction::NEUTRAL_COMMANDS || reaction == FaultReaction::SAFE ||
           reaction == FaultReaction::ERROR;
  }

  /**
   * @brief Values written on every update while a host is faulted with an internal reaction
   *        (HOLD, DAMP or SAFE; with NEUTRAL_COMMANDS the host keeps calling write(), with
   *        ERROR it is deactivated, which writes safe_values()). Called from the control loop,
   *        only with a supported reaction.
   *        Default: safe_values().
   * @param commands Latest command entries, e.g. the gains the policy wrote.
   * @param state Latest values of the state entries registered in configure().
   * @param reaction The reaction of the fault.
   * @param start true on the first call with this @p reaction, e.g. to capture the pose to hold.
   */
  virtual void fault(
    const DataStore & commands, const DataStore & state, std::vector<double> & values,
    FaultReaction reaction, bool start)
  {
    (void)commands;
    (void)state;
    (void)reaction;
    (void)start;
    safe_values(values);
  }
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__INTERFACES__INTERFACE_EXECUTOR_HPP_
