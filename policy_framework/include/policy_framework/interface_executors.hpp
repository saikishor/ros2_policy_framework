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

#ifndef POLICY_FRAMEWORK__INTERFACE_EXECUTORS_HPP_
#define POLICY_FRAMEWORK__INTERFACE_EXECUTORS_HPP_

#include <optional>
#include <string>
#include <vector>

#include "policy_framework/plugin_interfaces.hpp"

namespace policy_framework
{

/**
 * @brief Joint impedance in the controller: effort = kp (q_ref - q) + kd (dq_ref - dq), written
 *        to the `<joint>/<effort_interface>` command interfaces on every controller update.
 *
 * The same law as pid_controller with `reference_and_state_interfaces: [position, velocity]`,
 * no integral and no feedforward, so the policy can drive effort interfaces without a chained
 * pid_controller.
 *
 * - References: command entries `<joint>/position` and `<joint>/velocity` (0 when absent).
 * - Measurements: state entries `<joint>/position`, `<joint>/velocity`, registered so that the
 *   controller reads them from state interfaces on every update.
 * - Gains: command entries `<joint>/stiffness`, `<joint>/damping` when the policy writes them
 *   (e.g. MjlabPostprocessor with `gains_as_commands`), so they follow policy switches;
 *   otherwise the parameters.
 *
 * Parameters (under `<name>.`):
 * - `joints` (string array, default: every command entry ending in `/position`)
 * - `effort_interface` (default `effort`)
 * - `stiffness`, `damping` (double arrays, one per joint): fallback gains
 * - `effort_limits` (double array, one per joint, default empty = no clamping): |effort| bound
 * - `fault_damping` (double array, one per joint, default empty): the DAMP fault reaction
 *   writes effort = -fault_damping * velocity, within the effort limits (zero effort for a joint
 *   without a valid velocity); DAMP is only supported when every value is above zero
 *
 * Fault reactions: HOLD runs the PD law of hold() around the position at the fault (zero
 * effort for a joint without gains or position), DAMP the fault damping, SAFE zero effort.
 *
 * A joint whose reference, measurement or gains are not received or not finite is left
 * unchanged. safe_values() is zero effort.
 *
 * hold(): the same law around the joint positions of its start with a zero velocity reference,
 * with the gains the policy wrote as initial commands, otherwise the parameters; a joint without
 * gains is left unchanged.
 */
class PdEffortExecutor : public InterfaceExecutor
{
public:
  /// @throws std::runtime_error without gains for a joint, or on array size mismatch.
  void configure(const PluginContext & ctx, const DataStore & commands, DataStore & state) override;
  const std::vector<std::string> & command_interfaces() const override {return interfaces_;}
  /// Joint positions and velocities: the PD feedback.
  std::vector<std::string> state_entries() const override {return feedback_;}
  void write(const DataStore & commands, const DataStore & state, std::vector<double> & values) override;
  void hold(
    const DataStore & commands, const DataStore & state, std::vector<double> & values,
    bool start) override;
  void safe_values(std::vector<double> & values) override {values.assign(values.size(), 0.0);}
  /// @return true for NEUTRAL_COMMANDS, HOLD, SAFE and ERROR; for DAMP only when every joint
  ///         has a `fault_damping` above zero.
  bool supports_fault_reaction(FaultReaction reaction) const override;
  void fault(
    const DataStore & commands, const DataStore & state, std::vector<double> & values,
    FaultReaction reaction, bool start) override;

private:
  struct Joint
  {
    size_t ref_pos, q, dq;
    std::optional<size_t> ref_vel, kp_entry, kd_entry;
    double kp = 0.0, kd = 0.0, limit = 0.0;  // limit <= 0: none
    double fault_kd = 0.0;                   // the DAMP fault reaction
    bool has_kp = false, has_kd = false;     // the parameters give this gain
    double hold_q = 0.0;                     // NaN until hold() saw the position
  };
  /// Effort of @p joint towards (@p ref, @p ref_vel), NaN when something is missing.
  static double effort(
    const Joint & joint, std::optional<double> ref, std::optional<double> ref_vel,
    const DataStore & commands, const DataStore & state);

  std::vector<Joint> joints_;
  std::vector<std::string> interfaces_;
  std::vector<std::string> feedback_;
};

/**
 * @brief Copies command entries to command interfaces `<prefix>/<entry>`, e.g. the reference
 *        interfaces of a chained pid_controller (`<pid>/<joint>/position`).
 *
 * Parameters (under `<name>.`):
 * - `prefix` (default empty: the entry name is the interface name)
 * - `interfaces` (string array, default `[position, velocity]`): only entries
 *   `<joint>/<interface>` with one of these interfaces are written.
 *
 * Entries the policy never wrote are left unchanged; safe_values() leaves everything
 * unchanged (the chained controller keeps holding the last reference).
 */
class ReferenceInterfaceExecutor : public InterfaceExecutor
{
public:
  void configure(const PluginContext & ctx, const DataStore & commands, DataStore & state) override;
  const std::vector<std::string> & command_interfaces() const override {return interfaces_;}
  void write(const DataStore & commands, const DataStore & state, std::vector<double> & values) override;

private:
  std::vector<size_t> entries_;
  std::vector<std::string> interfaces_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__INTERFACE_EXECUTORS_HPP_
