// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/policy_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <optional>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/policy_controller_parameters.hpp"

namespace policy_framework
{

PolicyController::CallbackReturn PolicyController::on_init()
{
  return CallbackReturn::SUCCESS;
}

void PolicyController::load_executors()
{
  const auto node = get_node();
  executors_.clear();
  executor_loader_ = std::make_unique<pluginlib::ClassLoader<InterfaceExecutor>>(
    "policy_framework", "policy_framework::InterfaceExecutor");
  for (const auto & [name, type] : runner_->executor_types()) {
    if (!executor_loader_->isClassAvailable(type)) {
      throw std::runtime_error(
              "Executor '" + name + "' (" + type + ") is not an InterfaceExecutor plugin. Topic "
              "executors (e.g. MultiDofCommandExecutor) only run in policy_node.");
    }
    auto executor = executor_loader_->createSharedInstance(type);
    executor->configure({node, name}, runner_->commands(), runner_->state());
    executors_.push_back(executor);
    RCLCPP_INFO(node->get_logger(), "Executor '%s' (%s)", name.c_str(), type.c_str());
  }
}

PolicyController::CallbackReturn PolicyController::on_configure(const rclcpp_lifecycle::State &)
{
  inference_.reset();  // stops the thread before the runner it uses is replaced
  executors_.clear();
  runner_ = std::make_unique<PolicyRunner>();
  policy_controller::Params params;
  try {
    params = policy_controller::ParamListener(get_node()).get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  const double policy_rate = params.policy_rate;
  if (params.async_inference && is_async()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "async_inference and is_async are exclusive: async_inference "
      "already moves inference out of the real-time loop");
    return CallbackReturn::ERROR;
  }
  const double update_rate = static_cast<double>(get_update_rate());
  decimation_ = 1;
  if (policy_rate > 0.0) {
    const auto n = static_cast<unsigned int>(std::lround(update_rate / policy_rate));
    if (update_rate <= 0.0 || n < 1 || std::abs(n * policy_rate - update_rate) > 1e-6) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "policy_rate %.3f must divide the update_rate %.0f",
        policy_rate, update_rate);
      return CallbackReturn::ERROR;
    }
    decimation_ = n;
  }
  if (decimation_ > 1 && is_async()) {
    RCLCPP_WARN(
      get_node()->get_logger(), "is_async with policy_rate: executors (e.g. the PD law) also run "
      "in the asynchronous thread; prefer is_async: false");
  }
  try {
    runner_->configure(get_node(), false, get_robot_description());
    load_executors();
    state_names_ = runner_->unprovided_state();
    command_names_.clear();
    for (const auto & e : executors_) {
      for (const auto & name : e->state_entries()) {
        if (std::find(state_names_.begin(), state_names_.end(), name) == state_names_.end()) {
          throw std::runtime_error(
                  "State entry '" + name + "' is needed on every update by an executor and "
                  "must come from a state interface, but an input source provides it");
        }
      }
      for (const auto & name : e->command_interfaces()) {
        if (std::find(command_names_.begin(), command_names_.end(), name) !=
          command_names_.end())
        {
          throw std::runtime_error("Command interface '" + name + "' claimed twice");
        }
        command_names_.push_back(name);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (executors_.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "No executors: the commands are not sent anywhere");
  }
  if (params.async_inference) {
    // Allocated here, so that the control loop only copies values.
    async_state_ = runner_->state();
    async_commands_ = runner_->commands();
    inference_ =
      std::make_unique<realtime_tools::AsyncFunctionHandler<PolicyPipeline::Status>>();
    realtime_tools::AsyncFunctionHandlerParams async_params;
    async_params.thread_priority = static_cast<int>(params.inference_thread_priority);
    async_params.logger = get_node()->get_logger();  // e.g. its SCHED_FIFO warning
    async_params.wait_until_initial_trigger = false;
    inference_->init(
      [this](const rclcpp::Time &, const rclcpp::Duration &) {
        return runner_->step(async_state_, async_commands_);
      }, async_params);
    inference_->start_thread();
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "Claiming %zu state and %zu command interfaces; policy step "
    "every %u update(s)", state_names_.size(), command_names_.size(), decimation_);
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
  values_.clear();
  loaned_command_.clear();
  for (const auto & e : executors_) {
    std::vector<size_t> loaned;
    for (const auto & name : e->command_interfaces()) {
      size_t i = 0;
      while (i < command_interfaces_.size() && command_interfaces_[i].get_name() != name) {++i;}
      if (i == command_interfaces_.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Command interface '%s' not loaned", name.c_str());
        return CallbackReturn::ERROR;
      }
      loaned.push_back(i);
    }
    values_.emplace_back(loaned.size(), 0.0);
    loaned_command_.push_back(std::move(loaned));
  }
  policy_ready_ = false;
  updates_ = 0;
  inference_pending_ = false;
  runner_->pipeline().restart();
  return CallbackReturn::SUCCESS;
}

PolicyController::CallbackReturn PolicyController::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (inference_) {inference_->wait_for_trigger_cycle_to_finish();}
  inference_pending_ = false;
  write_safe_values();
  return CallbackReturn::SUCCESS;
}

void PolicyController::apply_values()
{
  for (size_t k = 0; k < executors_.size(); ++k) {
    for (size_t i = 0; i < values_[k].size(); ++i) {
      if (!std::isnan(values_[k][i])) {
        (void)command_interfaces_[loaned_command_[k][i]].set_value(values_[k][i]);
      }
    }
  }
}

void PolicyController::write_safe_values()
{
  for (size_t k = 0; k < executors_.size() && k < values_.size(); ++k) {
    executors_[k]->safe_values(values_[k]);
  }
  apply_values();
}

controller_interface::return_type PolicyController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Node clock (not the update time) so that it compares with the topic receive times.
  const double now = get_node()->now().seconds();
  auto & state = runner_->state();
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    // get_optional() fails only when the interface is locked by the writer (async mode): keep
    // the previous value then.
    if (const auto v = state_interfaces_[i].get_optional()) {state.set(state_idx_[i], *v, now);}
  }

  // The policy steps every decimation_ updates (starting with the first); executors run on
  // every update with the fresh state.
  const bool policy_tick = updates_++ % decimation_ == 0;
  if (policy_tick) {runner_->read_sources(now);}
  // On every update, so that other readers see the current joint state; before the step (or
  // the hand-off to the inference thread) so that the step sees this cycle's.
  runner_->update_kinematics();
  std::optional<PolicyPipeline::Status> status;  // of a step whose result arrived now
  if (!inference_) {
    if (policy_tick) {status = runner_->step();}
  } else {
    // Collect a finished step, then maybe trigger the next one; never wait for inference.
    if (inference_pending_ && !inference_->is_trigger_cycle_in_progress()) {
      inference_pending_ = false;
      status = inference_->get_last_return_value();
      if (status == PolicyPipeline::Status::OK) {
        runner_->commands().copy_values_from(async_commands_);
      }
    }
    if (policy_tick) {
      bool triggered = false;
      if (!inference_->is_trigger_cycle_in_progress()) {
        async_state_.copy_values_from(state);
        triggered = inference_->trigger_async_callback(time, period).first;
        inference_pending_ = triggered;
      }
      if (!triggered) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Inference is slower than policy_rate: skipping a policy step");
      }
    }
  }
  if (status == PolicyPipeline::Status::ERROR) {
    write_safe_values();
    return controller_interface::return_type::ERROR;
  }
  if (status == PolicyPipeline::Status::OK) {policy_ready_ = true;}
  // Before the first commands since activation, the command store may still hold the previous
  // activation's commands: send nothing.
  if (!policy_ready_) {return controller_interface::return_type::OK;}
  for (size_t k = 0; k < executors_.size(); ++k) {
    executors_[k]->write(runner_->commands(), state, values_[k]);
  }
  apply_values();
  return controller_interface::return_type::OK;
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::PolicyController, controller_interface::ControllerInterface)
