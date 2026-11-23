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

#include "policy_framework/hosts/policy_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <optional>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>

#include "pal_statistics/pal_statistics_macros.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/policy_controller_parameters.hpp"

namespace policy_framework
{

namespace
{
double elapsed_us(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start)
         .count();
}
}  // namespace

PolicyController::~PolicyController()
{
  // The global map would otherwise keep the registry, and its publisher thread reading our
  // members, alive after us. Its destructor joins that thread.
  DELETE_REGISTRY(introspection_key_);
}

PolicyController::CallbackReturn PolicyController::on_init()
{
  // Own registry rather than ros2_control's (shared by every controller): the values land on
  // this controller's topic, without the `<controller>.` prefix. Keyed by the resolved topic,
  // unique per controller.
  const std::string topic = "~/introspection_data";
  introspection_key_ = get_node()->get_node_topics_interface()->resolve_topic_name(topic);
  INITIALIZE_REGISTRY(get_node(), topic, introspection_key_);
  REGISTER_ENTITY(introspection_key_, "step_time_us", &timing_.step_time_us);
  REGISTER_ENTITY(introspection_key_, "preprocessor_time_us", &timing_.preprocessor_time_us);
  REGISTER_ENTITY(introspection_key_, "inference_time_us", &timing_.inference_time_us);
  REGISTER_ENTITY(introspection_key_, "postprocessor_time_us", &timing_.postprocessor_time_us);
  REGISTER_ENTITY(introspection_key_, "observation_age_ms", &timing_.observation_age_ms);
  REGISTER_ENTITY(introspection_key_, "update_time_us", &update_timing_.update_time_us);
  REGISTER_ENTITY(introspection_key_, "state_read_time_us", &update_timing_.state_read_time_us);
  REGISTER_ENTITY(
    introspection_key_, "read_sources_time_us", &update_timing_.read_sources_time_us);
  REGISTER_ENTITY(introspection_key_, "kinematics_time_us", &update_timing_.kinematics_time_us);
  REGISTER_ENTITY(introspection_key_, "safety_time_us", &update_timing_.safety_time_us);
  REGISTER_ENTITY(introspection_key_, "executors_time_us", &update_timing_.executors_time_us);
  START_PUBLISH_THREAD(introspection_key_);
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
  max_failed_reads_ = static_cast<unsigned int>(params.max_failed_reads);
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
    runner_->configure(get_node(), false, get_robot_description(), params.async_inference);
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
    // Refused now rather than discovered during a fall.
    for (size_t k = 0; k < executors_.size(); ++k) {
      for (const auto r : runner_->fault_reactions()) {
        if (!executors_[k]->supports_fault_reaction(r)) {
          throw std::runtime_error(
                  "Executor '" + runner_->executor_types()[k].first + "' cannot apply the fault "
                  "reaction '" + to_string(r) + "' of the safety configuration");
        }
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (executors_.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "No executors: the commands are not sent anywhere");
  }
  register_input_ages();
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
      const auto it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&](const auto & ci) {return ci.get_name() == name;});
      if (it == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Command interface '%s' not loaned", name.c_str());
        return CallbackReturn::ERROR;
      }
      loaned.push_back(static_cast<size_t>(it - command_interfaces_.begin()));
    }
    values_.emplace_back(loaned.size(), 0.0);
    loaned_command_.push_back(std::move(loaned));
  }
  policy_ready_ = false;
  hold_start_ = true;
  failed_reads_ = 0;
  timing_ = Timing();
  update_timing_ = UpdateTiming();
  updates_ = 0;
  inference_pending_ = false;
  observation_stamp_ = 0.0;
  fault_applied_.reset();
  std::fill(input_ages_ms_.begin(), input_ages_ms_.end(), 0.0);
  runner_->restart();
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
  for (size_t k = 0; k < values_.size(); ++k) {
    for (size_t i = 0; i < values_[k].size(); ++i) {
      if (!std::isnan(values_[k][i])) {
        (void)command_interfaces_[loaned_command_[k][i]].set_value(values_[k][i]);
      }
    }
  }
}

void PolicyController::register_input_ages()
{
  for (const auto & name : input_age_names_) {
    UNREGISTER_ENTITY(introspection_key_, "input_age_ms." + name);
  }
  input_age_names_.clear();
  input_age_offsets_.clear();
  for (const auto & p : runner_->policy_inputs()) {
    input_age_offsets_.push_back(input_age_names_.size());
    for (const auto & input : p.inputs) {input_age_names_.push_back(p.policy + "." + input);}
  }
  input_age_offsets_.push_back(input_age_names_.size());
  // Sized once, before registering: the registry keeps the addresses.
  input_ages_ms_.assign(input_age_names_.size(), 0.0);
  for (size_t i = 0; i < input_age_names_.size(); ++i) {
    REGISTER_ENTITY(introspection_key_, "input_age_ms." + input_age_names_[i], &input_ages_ms_[i]);
  }
}

void PolicyController::update_input_ages(double now)
{
  std::fill(input_ages_ms_.begin(), input_ages_ms_.end(), 0.0);  // inactive policies: unknown
  const auto & policies = runner_->policy_inputs();
  const auto & active = runner_->pipeline().active_policy();
  for (size_t k = 0; k < policies.size(); ++k) {
    if (policies[k].policy != active) {continue;}
    const auto & stamps = runner_->pipeline().input_stamps();
    const size_t n = std::min(stamps.size(), input_age_offsets_[k + 1] - input_age_offsets_[k]);
    for (size_t i = 0; i < n; ++i) {
      input_ages_ms_[input_age_offsets_[k] + i] =
        stamps[i].stamp > 0.0 ? (now - stamps[i].stamp) * 1e3 : 0.0;
    }
  }
}

double PolicyController::input_age_ms(const std::string & name) const
{
  const auto it = std::find(input_age_names_.begin(), input_age_names_.end(), name);
  return it == input_age_names_.end() ? std::numeric_limits<double>::quiet_NaN() :
         input_ages_ms_[static_cast<size_t>(it - input_age_names_.begin())];
}

controller_interface::return_type PolicyController::react(FaultReaction reaction)
{
  const auto start = std::chrono::steady_clock::now();
  if (reaction == FaultReaction::ERROR) {
    // The controller manager deactivates the controller (which writes the safe values again)
    // and activates its fallback controllers.
    write_safe_values();
    update_timing_.executors_time_us = elapsed_us(start);
    return controller_interface::return_type::ERROR;
  }
  const bool first = fault_applied_ != reaction;
  fault_applied_ = reaction;
  for (size_t k = 0; k < values_.size(); ++k) {
    executors_[k]->fault(runner_->commands(), runner_->state(), values_[k], reaction, first);
  }
  apply_values();
  update_timing_.executors_time_us = elapsed_us(start);
  return controller_interface::return_type::OK;
}

void PolicyController::write_safe_values()
{
  for (size_t k = 0; k < values_.size(); ++k) {
    executors_[k]->safe_values(values_[k]);
  }
  apply_values();
}

controller_interface::return_type PolicyController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto start = std::chrono::steady_clock::now();
  // Node clock (not the update time) so that it compares with the topic receive times.
  const double now = get_node()->now().seconds();
  // The previous update's values: every return path of the update is covered.
  // Real-time safe: copies the values for the publisher thread (skipped if it holds the lock).
  PUBLISH_ASYNC_STATISTICS(introspection_key_);
  update_timing_ = UpdateTiming();  // the items that do not run this time read 0
  const auto result = update_items(time, period, now);
  update_timing_.update_time_us = elapsed_us(start);
  return result;
}

controller_interface::return_type PolicyController::update_items(
  const rclcpp::Time & time, const rclcpp::Duration & period, double now)
{
  auto item_start = std::chrono::steady_clock::now();
  auto & state = runner_->state();
  bool read_failed = false;
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    // get_optional() fails only when the interface is locked by its writer (e.g. an
    // asynchronous hardware component): keep the previous value then, but not for long.
    if (const auto v = state_interfaces_[i].get_optional()) {
      state.set(state_idx_[i], *v, now);
    } else {
      read_failed = true;
    }
  }
  failed_reads_ = read_failed ? failed_reads_ + 1 : 0;
  update_timing_.state_read_time_us = elapsed_us(item_start);
  if (failed_reads_ > max_failed_reads_) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "State interfaces could not be read for %u updates in a row", failed_reads_);
    item_start = std::chrono::steady_clock::now();
    write_safe_values();
    update_timing_.executors_time_us = elapsed_us(item_start);
    return controller_interface::return_type::ERROR;
  }

  // The policy steps every decimation_ updates (starting with the first); executors run on
  // every update with the fresh state.
  const bool policy_tick = updates_++ % decimation_ == 0;
  if (policy_tick) {
    item_start = std::chrono::steady_clock::now();
    runner_->read_sources(now);
    update_timing_.read_sources_time_us = elapsed_us(item_start);
  }
  // On every update, so that other readers see the current joint state; before the step so
  // that it sees this cycle's (an inference thread updates its own model from its snapshot).
  item_start = std::chrono::steady_clock::now();
  runner_->update_kinematics();
  update_timing_.kinematics_time_us = elapsed_us(item_start);
  // Before the step: the policy must not act on a state that is already unsafe. While a
  // reaction is latched the policy is neither stepped nor followed (a step still running in the
  // inference thread is ignored).
  item_start = std::chrono::steady_clock::now();
  const auto reaction = runner_->update_safety(observation_stamp_, now);
  update_timing_.safety_time_us = elapsed_us(item_start);
  if (reaction && *reaction != FaultReaction::NEUTRAL_COMMANDS) {
    return react(*reaction);
  }
  std::optional<PolicyPipeline::Status> status;  // of a step whose result arrived now
  // Called right after a step finished, before the next one may start in the inference thread.
  auto record_step = [&]() {
      const auto & stages = runner_->pipeline().stage_times();
      timing_.preprocessor_time_us = stages.preprocessor_us;
      timing_.inference_time_us = stages.inference_us;
      timing_.postprocessor_time_us = stages.postprocessor_us;
      const double stamp = runner_->pipeline().observation_stamp();
      timing_.observation_age_ms =
        status == PolicyPipeline::Status::OK && stamp > 0.0 ? (now - stamp) * 1e3 : 0.0;
      if (status == PolicyPipeline::Status::OK) {
        observation_stamp_ = stamp;
        update_input_ages(now);
      }
    };
  if (!inference_) {
    if (policy_tick) {
      item_start = std::chrono::steady_clock::now();
      status = runner_->step();
      timing_.step_time_us = elapsed_us(item_start);
      record_step();
      if (status == PolicyPipeline::Status::OK) {
        if (const auto r = runner_->check_input_ages(now)) {return react(*r);}
      }
    }
  } else {
    // Collect a finished step, then maybe trigger the next one; never wait for inference.
    if (inference_pending_ && !inference_->is_trigger_cycle_in_progress()) {
      inference_pending_ = false;
      status = inference_->get_last_return_value();
      timing_.step_time_us =
        std::chrono::duration<double, std::micro>(inference_->get_last_execution_time()).count();
      record_step();
      if (status == PolicyPipeline::Status::OK) {
        // Before taking its commands: a stale input must not be acted on.
        if (const auto r = runner_->check_input_ages(now)) {return react(*r);}
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
          "Could not trigger a policy step (the inference thread is still running or busy): "
          "skipping it");
      }
    }
  }
  if (status == PolicyPipeline::Status::ERROR) {
    return react(runner_->policy_failure("The policy step failed"));
  }
  if (status == PolicyPipeline::Status::OK) {policy_ready_ = true;}
  // Before the first commands since activation the command store may still hold the previous
  // activation's references: the executors hold the robot instead of following them.
  item_start = std::chrono::steady_clock::now();
  for (size_t k = 0; k < executors_.size(); ++k) {
    if (policy_ready_) {
      executors_[k]->write(runner_->commands(), state, values_[k]);
    } else {
      executors_[k]->hold(runner_->commands(), state, values_[k], hold_start_);
    }
  }
  hold_start_ = false;
  apply_values();
  update_timing_.executors_time_us = elapsed_us(item_start);
  return controller_interface::return_type::OK;
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::PolicyController, controller_interface::ControllerInterface)
