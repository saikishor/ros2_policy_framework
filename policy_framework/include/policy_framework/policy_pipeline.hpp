// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__POLICY_PIPELINE_HPP_
#define POLICY_FRAMEWORK__POLICY_PIPELINE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "policy_framework/data_store.hpp"
#include "policy_framework/plugin_interfaces.hpp"

namespace policy_framework
{

/// One policy: its three stages plus switching behaviour.
struct Policy
{
  std::string name;
  std::shared_ptr<Preprocessor> preprocessor;
  std::shared_ptr<Inference> inference;
  std::shared_ptr<Postprocessor> postprocessor;
  /// > 0: after this many steps the pipeline switches back to the default (first) policy.
  int max_steps = 0;
  /// false: switch requests are rejected while this policy is active (e.g. a jump motion).
  bool interruptible = true;
};

/**
 * @brief Holds the configured policies and runs the active one, one step per control cycle.
 *
 * Switching is requested from any thread with request_switch() and applied at the beginning
 * of the next step(), so a cycle never mixes two policies. On activation the policy's
 * preprocessor and postprocessor are reset(). A policy keeps its memory (previous outputs,
 * preprocessor history) across a switch away and back, like pal_policy_deployer; it is only
 * cleared (outputs zeroed, Preprocessor::clear()) before its first activation and after
 * restart(). While a policy is not active its preprocessor observe()s every step instead, so
 * that this memory follows the robot.
 *
 * Policies must all be added before the first step(); step() itself does not allocate unless
 * the plugins do.
 */
class PolicyPipeline
{
public:
  enum class Status
  {
    OK,                ///< Commands were written.
    WAITING_FOR_DATA,  ///< The preprocessor is still missing inputs; no command written.
    ERROR              ///< A stage failed, the model answered a non-finite value, or there is
                       ///< no policy; no command written.
  };

  /**
   * @brief Add a configured policy. The first one added is the default, active at start.
   * @throws std::invalid_argument on duplicate name or missing stage.
   */
  void add_policy(Policy policy);

  /**
   * @brief Ask to activate @p name at the next step(). Thread-safe.
   * @param name Policy to activate. Requesting the active policy restarts it.
   * @param[out] message Human readable outcome.
   * @return false when @p name is unknown or the active policy is not interruptible.
   */
  bool request_switch(const std::string & name, std::string & message);

  /**
   * @brief Clear every policy and activate the default (first) one at the next step().
   *        Thread-safe.
   *
   * Bypasses the interruptible check; meant for (re)activation of the host, not for users.
   */
  void restart()
  {
    restart_requested_.store(true);
    requested_.store(0);
  }

  /// Postprocessor::initial_commands() of the default (first) policy. Not for the control cycle.
  void write_initial_commands(DataStore & commands)
  {
    if (!entries_.empty()) {entries_.front().policy.postprocessor->initial_commands(commands);}
  }

  /**
   * @brief Run one step of the active policy (applying any pending switch first).
   * @param state Inputs, read by the preprocessor.
   * @param commands Outputs, written by the postprocessor.
   */
  Status step(const DataStore & state, DataStore & commands);

  /**
   * @return Stamp of the observation of the last successful step: the oldest known input
   *         stamp of the active preprocessor, excluding inputs fed back from previous outputs
   *         and user commands (0 when unknown). The commands of that step carry this stamp. Read it from the
   *         thread calling step().
   */
  double observation_stamp() const {return observation_stamp_;}
  /// @return Input stamps of the active policy's last step. Read from the thread calling step().
  const InputStamps & input_stamps() const
  {
    return entries_.at(active_.load()).policy.preprocessor->input_stamps();
  }

  /// @return Name of the active policy. Thread-safe.
  const std::string & active_policy() const {return entries_.at(active_.load()).policy.name;}
  /// @return All policies, in the order they were added.
  std::vector<std::string> policy_names() const;

private:
  struct Entry
  {
    Policy policy;
    TensorMap inputs;
    TensorMap outputs;
    bool needs_clear = true;  ///< Only touched from step().
  };

  std::vector<Entry> entries_;
  std::atomic<size_t> active_{0};
  /// Pending switch request (-1: none), written by any thread, consumed by step().
  std::atomic<int> requested_{-1};
  std::atomic<bool> restart_requested_{false};
  // Only touched from step().
  bool needs_reset_ = true;
  int steps_ = 0;
  double observation_stamp_ = 0.0;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__POLICY_PIPELINE_HPP_
