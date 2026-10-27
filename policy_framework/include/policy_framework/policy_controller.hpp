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
 * its joint positions and IMU orientation are claimed like any other input.
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
 * Safety: executors run only once the active policy produced commands since activation. On
 * deactivation and when the policy fails, every executor's safe_values() are written (zero
 * effort for PdEffortExecutor).
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

private:
  /// Load the `executors` parameter as InterfaceExecutor plugins. @throws on any error.
  void load_executors();
  /// Write the executors' safe values.
  void write_safe_values();
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
  unsigned int decimation_ = 1;  ///< Updates per policy step.
  uint64_t updates_ = 0;         ///< Updates since activation.
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__POLICY_CONTROLLER_HPP_
