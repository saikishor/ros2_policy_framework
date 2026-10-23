// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/policy_runner.hpp"

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "policy_runner_parameters.hpp"

namespace policy_framework
{
namespace
{

template<typename T>
std::unique_ptr<pluginlib::ClassLoader<T>> make_loader(const std::string & base)
{
  return std::make_unique<pluginlib::ClassLoader<T>>("policy_framework", "policy_framework::" + base);
}

}  // namespace

void PolicyRunner::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, bool load_executors,
  const std::string & robot_description)
{
  logger_ = node->get_logger();
  const auto logger = node->get_logger();
  // Reconfiguration starts from scratch: drop plugins before their loaders.
  sources_.clear();
  executors_.clear();
  kinematics_.reset();
  pipeline_ = std::make_unique<PolicyPipeline>();
  state_ = DataStore();
  commands_ = DataStore();
  source_loader_ = make_loader<InputSource>("InputSource");
  preprocessor_loader_ = make_loader<Preprocessor>("Preprocessor");
  inference_loader_ = make_loader<Inference>("Inference");
  postprocessor_loader_ = make_loader<Postprocessor>("Postprocessor");
  executor_loader_ = make_loader<Executor>("Executor");

  // Declares and validates sources, policies, executors and their per-name parameters.
  const auto params = policy_runner::ParamListener(node).get_params();
  executor_types_.clear();
  for (const auto & name : params.executors) {
    executor_types_.emplace_back(name, params.executors_map.at(name).type);
  }

  // Before the policies: their plugins request frames while configuring.
  kinematics_ = Kinematics::create(node, robot_description, state_);
  if (kinematics_) {
    RCLCPP_INFO(logger, "Kinematics over %zu joints", kinematics_->joint_names().size());
  }

  for (const auto & name : params.sources) {
    const auto & type = params.sources_map.at(name).type;
    auto source = source_loader_->createSharedInstance(type);
    source->configure({node, name});
    sources_.push_back(source);
    RCLCPP_INFO(logger, "Input source '%s' (%s)", name.c_str(), type.c_str());
  }

  for (const auto & name : params.policies) {
    const PluginContext ctx{node, name, kinematics_};
    const auto & stages = params.policies_map.at(name);
    Policy p;
    p.name = name;
    p.inference = inference_loader_->createSharedInstance(stages.inference);
    p.inference->configure(ctx);
    p.preprocessor = preprocessor_loader_->createSharedInstance(stages.preprocessor);
    p.preprocessor->configure(ctx, p.inference->model(), state_);
    p.postprocessor = postprocessor_loader_->createSharedInstance(stages.postprocessor);
    p.postprocessor->configure(ctx, p.inference->model(), commands_);
    p.max_steps = static_cast<int>(stages.max_steps);
    p.interruptible = stages.interruptible;
    RCLCPP_INFO(
      logger, "Policy '%s': max_steps=%d interruptible=%s", name.c_str(), p.max_steps,
      p.interruptible ? "true" : "false");
    pipeline_->add_policy(std::move(p));
  }

  for (const auto & [name, type] : load_executors ? executor_types_ : decltype(executor_types_){}) {
    auto executor = executor_loader_->createSharedInstance(type);
    executor->configure({node, name}, commands_);
    executors_.push_back(executor);
    RCLCPP_INFO(logger, "Executor '%s' (%s)", name.c_str(), type.c_str());
  }

  switch_service_ = node->create_service<srv::SwitchPolicy>(
    "~/switch_policy",
    [this](
      const std::shared_ptr<srv::SwitchPolicy::Request> req,
      std::shared_ptr<srv::SwitchPolicy::Response> res) {
      res->success = pipeline_->request_switch(req->policy_name, res->message);
      RCLCPP_INFO(logger_, "switch_policy: %s", res->message.c_str());
    });

  last_status_ = PolicyPipeline::Status::OK;
  last_policy_.clear();
  RCLCPP_INFO(
    logger, "%zu state entries, %zu command entries, active policy '%s'", state_.size(),
    commands_.size(), pipeline_->active_policy().c_str());
}

std::vector<std::string> PolicyRunner::unprovided_state() const
{
  std::vector<std::string> missing;
  for (const auto & name : state_.names()) {
    bool provided = false;
    for (const auto & s : sources_) {
      if (s->provides(name)) {
        provided = true;
        break;
      }
    }
    if (!provided) {missing.push_back(name);}
  }
  return missing;
}

void PolicyRunner::read_sources(double now)
{
  for (const auto & s : sources_) {s->read(state_, now);}
}

PolicyPipeline::Status PolicyRunner::step(const DataStore & state, DataStore & commands)
{
  PolicyPipeline::Status status;
  try {
    if (kinematics_) {kinematics_->update(state);}
    status = pipeline_->step(state, commands);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Policy step failed: %s", e.what());
    status = PolicyPipeline::Status::ERROR;
  }
  // ponytail: logs from the control cycle on transitions only (rare), not real-time safe; move
  // to a realtime publisher if a robot ever shows jitter on switches.
  const auto & active = pipeline_->active_policy();
  if (active != last_policy_) {
    RCLCPP_INFO(logger_, "Active policy: '%s'", active.c_str());
    last_policy_ = active;
  }
  if (status != last_status_) {
    if (status == PolicyPipeline::Status::WAITING_FOR_DATA) {
      RCLCPP_WARN(logger_, "Waiting for input data of policy '%s'", active.c_str());
    } else if (status == PolicyPipeline::Status::OK) {
      RCLCPP_INFO(logger_, "Policy '%s' is running", active.c_str());
    }
    last_status_ = status;
  }
  return status;
}

void PolicyRunner::write_executors()
{
  for (const auto & e : executors_) {e->write(commands_);}
}

}  // namespace policy_framework
