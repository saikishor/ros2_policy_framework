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


#ifndef POLICY_FRAMEWORK__INTERFACES__SAFETY_CHECK_HPP_
#define POLICY_FRAMEWORK__INTERFACES__SAFETY_CHECK_HPP_

#include <optional>
#include <string>
#include <vector>

#include "policy_framework/core/data_store.hpp"
#include "policy_framework/interfaces/plugin_context.hpp"
#include "policy_framework/interfaces/types.hpp"

/// @file safety_check.hpp
/// @brief Base class of the SafetyCheck plugins.

namespace policy_framework
{

/**
 * @brief A check of the measured state that tells whether the robot is in a state it must not
 *        be in (e.g. an actuator reporting an error, a motor too hot, the centre of mass outside
 *        the feet).
 *
 * The contract: configure() registers every state entry the check reads, check() is then
 * called with the state on every cycle of the control loop and returns a Verdict that is not
 * safe when the state is unsafe, and reset() forgets what the check accumulated (filters,
 * counters).
 *
 * What to do about a failure is configured per instance (`on_fault`), not decided by the check;
 * a failing check may ask for another reaction in its Verdict, among the ones it declares once
 * configured (possible_reactions()), e.g. a more severe one when several actuators fail.
 *
 * PolicyRunner loads them from its `safety_checks` parameter (see PolicyRunner::update_safety()).
 */
class SafetyCheck
{
public:
  /// The outcome of check(): no allocation, so that it can be returned from the control loop.
  struct Verdict
  {
    bool safe = true;
    /// When not safe: the reaction asked for, one of possible_reactions(); nothing: the
    /// `on_fault` configured for this check.
    std::optional<FaultReaction> reaction;
  };

  virtual ~SafetyCheck() = default;

  /**
   * @brief Register the state entries the check reads (state.add()).
   * @param ctx Parameters live under `<ctx.name>.`; ctx.kinematics is a whole-body model of the
   *        state (null when there is none).
   * @param commands Command entries of the policies (e.g. to find the commanded joints).
   * @param state Store the checked entries are registered in.
   * @throws std::exception on invalid parameters.
   */
  virtual void configure(
    const PluginContext & ctx, const DataStore & commands, DataStore & state) = 0;

  /**
   * @brief Check the state. Runs in the control loop: must not allocate while it passes.
   * @param state The store given to configure(), with the current values.
   * @param now Current time in seconds, on the clock of the state stamps.
   * @param[out] reason Why it failed; written only on failure (so that passing never
   *             allocates).
   * @return Whether the state is safe and, when it is not, the reaction asked for (e.g. a more
   *         severe one when several actuators fail at once).
   */
  virtual Verdict check(const DataStore & state, double now, std::string & reason) = 0;

  /**
   * @return The reactions a Verdict may ask for, besides the `on_fault` configured for this
   *         check, known once configured: hosts refuse while configuring one that an executor
   *         does not support. Default: none.
   */
  virtual std::vector<FaultReaction> possible_reactions() const {return {};}

  /// Forget filters, counters and a persisting failure, so that a new run does not inherit the
  /// previous one's: on host reactivation, on `~/reset_safety`, and when re-enabled by a policy.
  virtual void reset() {}
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__INTERFACES__SAFETY_CHECK_HPP_
