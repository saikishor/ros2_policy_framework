// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__POLICY_CONTROLLER_HPP_
#define POLICY_FRAMEWORK__POLICY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "policy_framework/plugin_interfaces.hpp"
#include "policy_framework/policy_runner.hpp"
#include "realtime_tools/async_function_handler.hpp"

namespace policy_framework
{

/**
 * @brief ros2_control controller running the policies of a PolicyRunner.
 *
 * Inputs: every state entry required by the policies or the executors that no input source
 * (topic) provides is claimed as a state interface of the same name (e.g. `joint1/position`,
 * `imu/orientation.x`). So policy inputs can come from topics only, state interfaces only, or
 * a mix. Entries an executor needs on every update (state_entries(), e.g. PD feedback) must
 * come from state interfaces.
 *
 * Kinematics (`kinematics.*`, see Kinematics) uses the controller manager's robot description;
 * its joint positions and velocities are claimed like any other input. kinematics() follows
 * the state interfaces on every update; with `async_inference` the policies' plugins use a
 * second model, computed in the inference thread from the state snapshot of their step, so
 * that the frames they read always belong to the joint state they read.
 *
 * Outputs: `executors` are InterfaceExecutor plugins (each needs `<name>.type`) that turn the
 * commands into command interface values, e.g. policy_framework/PdEffortExecutor (PD law into
 * efforts) or policy_framework/ReferenceInterfaceExecutor (chain into a pid_controller). Topic
 * Executors only run in the standalone policy_node and are rejected here.
 *
 * Timing: `policy_rate` (Hz, default 0 = every update) runs the policy every
 * update_rate / policy_rate updates (must be an integer), while the executors run on every
 * update. E.g. update_rate 1000 and policy_rate 50 for a PD law at 1 kHz with a 50 Hz policy;
 * without executors needing more, update_rate = policy rate. Synchronous by default; with the
 * standard parameter `is_async: true` update() (inference and executors) runs in its own
 * thread, which suits a reference executor but not a PD law.
 *
 * `async_inference: true` keeps update() and the executors in the real-time loop and runs only
 * the policy step in its own thread (SCHED_FIFO `inference_thread_priority`): every policy
 * tick the state is handed over and the step triggered; update() never waits for it, and its
 * commands apply from the first update after it finished (until then the previous ones are
 * held). A tick that finds the previous step still running is skipped with a warning.
 * Exclusive with is_async.
 *
 * Faults (a failed policy step, e.g. a non-finite model output; state interfaces unreadable
 * for more than `max_failed_reads` updates; a violated `safety.*` check, see SafetyMonitor)
 * are handled per `on_fault`: `stop` (default) writes every executor's safe_values() and
 * returns ERROR; `damp` keeps the controller active, stops following the policy and writes
 * every executor's damp() values on every update until the controller is reactivated.
 *
 * Safety: from activation until the active policy produced its first commands (it may wait for
 * its inputs, or for its first inference) the executors hold() the robot where it is
 * (PdEffortExecutor: a PD around the position at activation); after that, a step waiting for
 * data keeps the last commands applied. On deactivation, when the policy fails, and when a
 * state interface could not be read for more than `max_failed_reads` updates in a row (the
 * previous value is used meanwhile), every executor's safe_values() are written (zero effort
 * for PdEffortExecutor).
 *
 * Introspection: the duration of each policy step and the age of its observation (see Timing)
 * are registered with ros2_control's introspection.
 *
 * On activation the first policy restarts from scratch. Policies are switched with the
 * `~/switch_policy` service.
 */
class PolicyController : public controller_interface::ControllerInterface
{
public:
  CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Measurements of the last finished policy step, also published by ros2_control's
  /// introspection as `<controller>.policy_step_time_us` and `.observation_age_ms`.
  struct Timing
  {
    /// Duration of the step (preprocessor, inference, postprocessor), microseconds.
    double step_time_us = 0.0;
    /// Age of the oldest input of the step when its commands were applied, milliseconds: how
    /// stale the observation behind the current commands was. 0 when unknown.
    double observation_age_ms = 0.0;
  };
  const Timing & timing() const {return timing_;}

  /**
   * @return The whole-body kinematics (null unless `kinematics.enabled`), updated on every
   *         update() from the state interfaces. Its readers are thread-safe.
   */
  std::shared_ptr<Kinematics> kinematics() const
  {
    return runner_ ? runner_->kinematics() : nullptr;
  }

private:
  /// Load the `executors` parameter as InterfaceExecutor plugins. @throws on any error.
  void load_executors();
  /// Write the executors' safe values.
  void write_safe_values();
  /**
   * @brief Handle a fault per `on_fault`: safe values and ERROR, or latch the fault, write the
   *        executors' damping values and keep going. Logs @p reason (once when latched).
   */
  controller_interface::return_type fault(const std::string & reason);
  /// Write values[k] of executor k to its loaned interfaces (NaN: leave unchanged).
  void apply_values();

  // Declared before the runner and executors: plugin libraries must outlive the instances.
  std::unique_ptr<pluginlib::ClassLoader<InterfaceExecutor>> executor_loader_;
  std::unique_ptr<PolicyRunner> runner_;
  // After runner_: destroyed (thread stopped) before the runner its callback uses.
  std::unique_ptr<realtime_tools::AsyncFunctionHandler<PolicyPipeline::Status>> inference_;
  DataStore async_state_;     ///< Inference thread's copy of the state, filled per policy tick.
  DataStore async_commands_;  ///< Inference thread's commands, copied out when it finished.
  bool inference_pending_ = false;  ///< Triggered, and its result not applied yet.
  std::vector<std::shared_ptr<InterfaceExecutor>> executors_;
  std::vector<std::string> state_names_;
  std::vector<std::string> command_names_;  ///< Concatenation of the executors' interfaces.
  // Filled on activation.
  std::vector<size_t> state_idx_;           ///< Store index of each loaned state interface.
  std::vector<std::vector<double>> values_;           ///< Per executor, one per interface.
  std::vector<std::vector<size_t>> loaned_command_;   ///< Per executor, loaned interface index.
  bool policy_ready_ = false;  ///< The policy produced commands since activation.
  bool damp_on_fault_ = false;  ///< `on_fault: damp`.
  bool faulted_ = false;        ///< A fault latched with `on_fault: damp` since activation.
  double observation_stamp_ = 0.0;  ///< Of the last successful policy step since activation.
  bool hold_start_ = true;     ///< No update since activation yet.
  unsigned int max_failed_reads_ = 5;
  unsigned int failed_reads_ = 0;  ///< Consecutive updates with an unreadable state interface.
  Timing timing_;
  unsigned int decimation_ = 1;  ///< Updates per policy step.
  uint64_t updates_ = 0;         ///< Updates since activation.
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__POLICY_CONTROLLER_HPP_
