// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__POLICY_CONTROLLER_HPP_
#define POLICY_FRAMEWORK__POLICY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "policy_framework/policy_runner.hpp"

namespace policy_framework
{

/**
 * @brief ros2_control controller running the policies of a PolicyRunner.
 *
 * Inputs: every state entry required by the policies that no input source (topic) provides is
 * claimed as a state interface of the same name (e.g. `joint1/position`,
 * `imu/orientation.x`). So inputs can come from topics only, state interfaces only, or a mix.
 *
 * Outputs: with `use_command_interfaces` (default true) every command entry is claimed as the
 * command interface `<command_interface_prefix>/<entry>` (prefix default empty), e.g.
 * `rl_impedance_controller/joint1/position` to chain into a pid_controller. Executors
 * (topics) can be used in addition or instead.
 *
 * Only entries a policy actually wrote are sent to the command interfaces.
 *
 * Timing: one policy step per update(), so `update_rate` must be the policy rate. Synchronous
 * by default; set the standard controller parameter `is_async: true` to run update() (hence
 * inference) in its own thread instead of the controller manager's real-time loop.
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
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<PolicyRunner> runner_;
  std::vector<std::string> state_names_;
  std::vector<std::string> command_names_;  ///< Command interface names (with prefix).
  std::string command_prefix_;
  bool use_command_interfaces_ = true;
  // Store index of each loaned interface, filled on activation.
  std::vector<size_t> state_idx_;
  std::vector<size_t> command_idx_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__POLICY_CONTROLLER_HPP_
