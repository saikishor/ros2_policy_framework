// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/policy_controller.hpp"

#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace policy_framework
{

PolicyController::CallbackReturn PolicyController::on_init()
{
  auto_declare<std::string>("command_interface_prefix", "");
  auto_declare<bool>("use_command_interfaces", true);
  return CallbackReturn::SUCCESS;
}

PolicyController::CallbackReturn PolicyController::on_configure(const rclcpp_lifecycle::State &)
{
  command_prefix_ = get_node()->get_parameter("command_interface_prefix").as_string();
  use_command_interfaces_ = get_node()->get_parameter("use_command_interfaces").as_bool();
  runner_ = std::make_unique<PolicyRunner>();
  try {
    runner_->configure(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  state_names_ = runner_->unprovided_state();
  command_names_.clear();
  if (use_command_interfaces_) {
    for (const auto & name : runner_->commands().names()) {
      command_names_.push_back(command_prefix_.empty() ? name : command_prefix_ + "/" + name);
    }
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "Claiming %zu state and %zu command interfaces",
    state_names_.size(), command_names_.size());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PolicyController::command_interface_configuration() const
{
  if (command_names_.empty()) {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, command_names_};
}

controller_interface::InterfaceConfiguration
PolicyController::state_interface_configuration() const
{
  if (state_names_.empty()) {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, state_names_};
}

PolicyController::CallbackReturn PolicyController::on_activate(const rclcpp_lifecycle::State &)
{
  // Map by name: do not rely on the order in which the controller manager loans interfaces.
  state_idx_.clear();
  for (const auto & si : state_interfaces_) {
    state_idx_.push_back(runner_->state().index(si.get_name()));
  }
  command_idx_.clear();
  const auto prefix_len = command_prefix_.empty() ? 0 : command_prefix_.size() + 1;
  for (const auto & ci : command_interfaces_) {
    command_idx_.push_back(runner_->commands().index(ci.get_name().substr(prefix_len)));
  }
  runner_->pipeline().restart();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type PolicyController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Node clock (not the update time) so that it compares with the topic receive times.
  const double now = get_node()->now().seconds();
  runner_->read_sources(now);
  auto & state = runner_->state();
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    // get_optional() fails only when the interface is locked by the writer (async mode): keep
    // the previous value then.
    if (const auto v = state_interfaces_[i].get_optional()) {state.set(state_idx_[i], *v, now);}
  }

  const auto status = runner_->step();
  if (status == PolicyPipeline::Status::ERROR) {
    return controller_interface::return_type::ERROR;
  }
  if (status != PolicyPipeline::Status::OK) {
    return controller_interface::return_type::OK;
  }
  const auto & commands = runner_->commands();
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (commands.received(command_idx_[i])) {
      (void)command_interfaces_[i].set_value(commands.get(command_idx_[i]));
    }
  }
  runner_->write_executors();
  return controller_interface::return_type::OK;
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::PolicyController, controller_interface::ControllerInterface)
