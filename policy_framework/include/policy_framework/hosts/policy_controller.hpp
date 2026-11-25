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

#ifndef POLICY_FRAMEWORK__HOSTS__POLICY_CONTROLLER_HPP_
#define POLICY_FRAMEWORK__HOSTS__POLICY_CONTROLLER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "policy_framework/plugin_interfaces.hpp"
#include "policy_framework/hosts/policy_runner.hpp"
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
 * held). A tick that finds the previous step still running, or the inference thread busy for
 * an instant (the trigger never waits for its lock), is skipped with a warning.
 * Exclusive with is_async.
 *
 * Safety: from activation until the active policy produced its first commands (it may wait for
 * its inputs, or for its first inference) the executors hold() the robot where it is
 * (PdEffortExecutor: a PD around the position at activation); after that, a step waiting for
 * data keeps the last commands applied. On deactivation, when the policy fails, and when a
 * state interface could not be read for more than `max_failed_reads` updates in a row (the
 * previous value is used meanwhile), every executor's safe_values() are written (zero effort
 * for PdEffortExecutor).
 *
 * Introspection: the durations of each policy step and of its stages and the age of its
 * observation (see Timing), the duration of each item of update() (see UpdateTiming), and the
 * age of each input of the active policy (see input_age_ms()), are published by the controller itself (pal_statistics) on
 * `~/introspection_data/{names,values,full}`, e.g. `/<controller>/introspection_data/full`,
 * on every update. Not on the controller manager's shared introspection topic.
 *
 * On activation the first policy restarts from scratch. Policies are switched with the
 * `~/switch_policy` service, listed by `~/list_policies`, and the running one is published on
 * `~/active_policies`. `~/reset_safety` clears a latched hold, damp or safe without
 * reactivation (PolicyRunner::apply_safety_reset()); an error hands over to the fallback
 * controllers, and only a reactivation brings the controller back.
 */
class PolicyController : public controller_interface::ControllerInterface
{
public:
  /// Deletes the introspection registry from pal_statistics' global map (see on_init()).
  ~PolicyController() override;
  /// Creates the introspection registry, keyed by its resolved topic
  /// (`/<controller>/introspection_data`) in pal_statistics' global map, and starts its
  /// publisher thread.
  CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Measurements of the last finished policy step, also published on the controller's
  /// introspection topic under the names of the fields (e.g. `inference_time_us`).
  struct Timing
  {
    /// Total duration of the step, microseconds: its stages plus the work around them
    /// (policy switch, inactive policies observing), so at least the sum of the three below.
    double step_time_us = 0.0;
    /// Durations of the step's stages, microseconds (see PolicyPipeline::stage_times()); 0
    /// for a stage that did not run (e.g. inference while the preprocessor waits for data).
    double preprocessor_time_us = 0.0;
    double inference_time_us = 0.0;
    double postprocessor_time_us = 0.0;
    /// Age of the oldest input of the step when its commands were applied, milliseconds: how
    /// stale the observation behind the current commands was. 0 when unknown.
    double observation_age_ms = 0.0;
  };
  const Timing & timing() const {return timing_;}

  /// Durations of the items of the last update(), microseconds, also published on the
  /// controller's introspection topic under the names of the fields. An item that did not run
  /// (e.g. after an early return on a fault) is 0. The policy step is in Timing.
  struct UpdateTiming
  {
    /// Whole update(), including the policy step when synchronous (with `async_inference`
    /// only the hand-over to the inference thread).
    double update_time_us = 0.0;
    double state_read_time_us = 0.0;    ///< Reading the state interfaces.
    double read_sources_time_us = 0.0;  ///< Reading the input sources (policy ticks only).
    double kinematics_time_us = 0.0;    ///< Updating the kinematics from the state.
    double safety_time_us = 0.0;        ///< Evaluating the safety checks.
    /// The executors computing and writing the command interfaces: write(), hold(), or a
    /// fault reaction's fault() or safe_values().
    double executors_time_us = 0.0;
  };
  const UpdateTiming & update_timing() const {return update_timing_;}

  /**
   * @return Age in milliseconds of the input `<policy>.<input>` (a term or an entry, see
   *         PolicyRunner::policy_inputs()) when the commands of the last step were applied: now
   *         minus its stamp. 0 for the inputs of an inactive policy or when unknown, NaN for a
   *         name that is no input. Also published on the controller's introspection topic as
   *         `input_age_ms.<policy>.<input>`. Not real-time (looks the name up).
   */
  double input_age_ms(const std::string & name) const;

  /**
   * @return The whole-body kinematics (null unless `kinematics.enabled`), updated on every
   *         update() from the state interfaces. Its readers are thread-safe.
   */
  std::shared_ptr<Kinematics> kinematics() const
  {
    return runner_ ? runner_->kinematics() : nullptr;
  }

private:
  /// The body of update(), timed by it.
  controller_interface::return_type update_items(
    const rclcpp::Time & time, const rclcpp::Duration & period, double now);
  /// Load the `executors` parameter as InterfaceExecutor plugins. @throws on any error.
  void load_executors();
  /// Apply a latched fault @p reaction: the executors' fault() values, or for ERROR the safe
  /// values and an error for the controller manager.
  controller_interface::return_type react(FaultReaction reaction);
  /// Write the executors' safe values.
  void write_safe_values();
  /// Forget the previous run of the policy (when activated, and when the safety is reset): the
  /// executors hold until a fresh step, a pending step is dropped, the observation is unknown.
  void start_over();
  /// Register one introspection value per input of every policy (and drop the previous ones).
  void register_input_ages();
  /// Update the input ages from the stamps of the step that just finished.
  void update_input_ages(double now);
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
  bool hold_start_ = true;     ///< No update since activation yet.
  unsigned int max_failed_reads_ = 5;
  unsigned int failed_reads_ = 0;  ///< Consecutive updates with an unreadable state interface.
  Timing timing_;
  UpdateTiming update_timing_;
  std::vector<std::string> input_age_names_;  ///< `<policy>.<input>`, every policy's inputs.
  std::vector<double> input_ages_ms_;         ///< Same order; registered: never reallocated.
  std::vector<size_t> input_age_offsets_;     ///< Index of each policy's first input, + end.
  unsigned int decimation_ = 1;  ///< Updates per policy step.
  double observation_stamp_ = 0.0;  ///< Of the last successful step, for safety.max_input_age.
  std::optional<FaultReaction> fault_applied_;  ///< Reaction of the previous update, if any.
  uint64_t updates_ = 0;         ///< Updates since activation.
  /// pal_statistics registry key of `~/introspection_data`, which publishes timing_,
  /// update_timing_ and input_ages_ms_; the registry is deleted in the destructor, before those values.
  std::string introspection_key_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__HOSTS__POLICY_CONTROLLER_HPP_
