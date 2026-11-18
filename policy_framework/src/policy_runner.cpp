// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/policy_runner.hpp"

#include <algorithm>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "policy_framework/kinematics_parameters.hpp"
#include "policy_framework/policy_runner_parameters.hpp"

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
  const std::string & robot_description, bool step_in_other_thread)
{
  logger_ = node->get_logger();
  const auto & logger = logger_;
  // Reconfiguration starts from scratch: drop plugins before their loaders.
  sources_.clear();
  executors_.clear();
  safety_checks_.clear();
  kinematics_ = KinematicsFeed();
  policy_kinematics_ = KinematicsFeed();
  pipeline_ = std::make_unique<PolicyPipeline>();
  state_ = DataStore();
  commands_ = DataStore();
  source_loader_ = make_loader<InputSource>("InputSource");
  preprocessor_loader_ = make_loader<Preprocessor>("Preprocessor");
  inference_loader_ = make_loader<Inference>("Inference");
  postprocessor_loader_ = make_loader<Postprocessor>("Postprocessor");
  executor_loader_ = make_loader<Executor>("Executor");
  safety_check_loader_ = make_loader<SafetyCheck>("SafetyCheck");

  // Declares and validates sources, policies, executors and their per-name parameters.
  const auto params = policy_runner::ParamListener(node).get_params();
  executor_types_.clear();
  for (const auto & name : params.executors) {
    executor_types_.emplace_back(name, params.executors_map.at(name).type);
  }

  const auto & safety = params.safety_entry;
  if (!safety.empty()) {state_.add(safety);}
  command_gate_ = CommandGate(PluginContext{node, "", nullptr, safety});
  commands_accepted_ = true;

  // Before the policies: their plugins look frames up while configuring.
  configure_kinematics(node, robot_description, step_in_other_thread);

  for (const auto & name : params.sources) {
    const auto & type = params.sources_map.at(name).type;
    auto source = source_loader_->createSharedInstance(type);
    source->configure({node, name, nullptr, safety});
    sources_.push_back(source);
    RCLCPP_INFO(logger, "Input source '%s' (%s)", name.c_str(), type.c_str());
  }

  for (const auto & name : params.policies) {
    const PluginContext ctx{node, name, policy_kinematics_.model, safety};
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

  SafetyMonitor::Config safety_config;
  safety_config.max_tilt = params.safety.max_tilt;
  safety_config.imu = params.safety.imu;
  safety_config.max_joint_velocity = params.safety.max_joint_velocity;
  safety_config.position_margin = params.safety.position_margin;
  safety_config.max_input_age = params.safety.max_input_age;
  if (params.safety.joint_position_limits) {
    if (robot_description.empty()) {
      throw std::runtime_error("safety.joint_position_limits needs a robot description");
    }
    safety_config.joint_limits = SafetyMonitor::joint_limits(robot_description);
  }
  safety_.configure(safety_config, commands_, state_);

  // After the policies: a check may look at the commanded entries.
  for (const auto & name : params.safety_checks) {
    const auto & type = params.safety_checks_map.at(name).type;
    auto check = safety_check_loader_->createSharedInstance(type);
    check->configure({node, name, kinematics_.model, safety}, commands_, state_);
    safety_checks_.emplace_back(name, check);
    RCLCPP_INFO(logger, "Safety check '%s' (%s)", name.c_str(), type.c_str());
  }

  if (load_executors) {
    for (const auto & [name, type] : executor_types_) {
      auto executor = executor_loader_->createSharedInstance(type);
      executor->configure({node, name}, commands_);
      executors_.push_back(executor);
      RCLCPP_INFO(logger, "Executor '%s' (%s)", name.c_str(), type.c_str());
    }
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

std::optional<std::string> PolicyRunner::check_safety(double observation_stamp, double now)
{
  if (const auto v = safety_.check(state_, observation_stamp, now)) {return v->describe();}
  for (const auto & [name, check] : safety_checks_) {
    std::string reason;  // empty: no allocation while the check passes
    if (!check->check(state_, now, reason)) {return name + ": " + reason;}
  }
  return std::nullopt;
}

std::vector<std::string> PolicyRunner::unprovided_state() const
{
  std::vector<std::string> missing;
  for (const auto & name : state_.names()) {
    if (std::none_of(
        sources_.begin(), sources_.end(), [&](const auto & s) {return s->provides(name);}))
    {
      missing.push_back(name);
    }
  }
  return missing;
}

void PolicyRunner::configure_kinematics(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & robot_description,
  bool step_in_other_thread)
{
  kinematics_position_idx_.clear();
  kinematics_velocity_idx_.clear();
  const auto params = kinematics::ParamListener(node, "kinematics").get_params();
  if (!params.enabled) {return;}
  if (robot_description.empty()) {
    throw std::runtime_error("kinematics.enabled needs a robot description");
  }
  auto make = [&]() {
      KinematicsFeed feed;
      feed.model = std::make_shared<Kinematics>(robot_description, params.joints);
      const auto n = static_cast<Eigen::Index>(feed.model->joint_names().size());
      feed.q = Eigen::VectorXd::Zero(n);
      feed.v = Eigen::VectorXd::Zero(n);
      return feed;
    };
  kinematics_ = make();
  // A step in another thread reads a copy of the state: its plugins need the kinematics of
  // that copy, not of whatever the host's thread has moved on to meanwhile.
  policy_kinematics_ = step_in_other_thread ? make() : kinematics_;
  for (const auto & j : kinematics_.model->joint_names()) {
    kinematics_position_idx_.push_back(state_.add(j + "/position"));
    kinematics_velocity_idx_.push_back(state_.add(j + "/velocity"));
  }
  RCLCPP_INFO(logger_, "Kinematics over %zu joints", kinematics_.model->joint_names().size());
}

void PolicyRunner::read_sources(double now)
{
  for (const auto & s : sources_) {s->read(state_, now);}
}

void PolicyRunner::update_kinematics(KinematicsFeed & feed, const DataStore & state) const
{
  if (!feed.model || !state.all_received(kinematics_position_idx_) ||
    !state.all_received(kinematics_velocity_idx_))
  {
    return;
  }
  double stamp = 0.0;
  for (size_t j = 0; j < kinematics_position_idx_.size(); ++j) {
    const auto p = kinematics_position_idx_[j];
    const auto v = kinematics_velocity_idx_[j];
    feed.q[static_cast<Eigen::Index>(j)] = state.get(p);
    feed.v[static_cast<Eigen::Index>(j)] = state.get(v);
    for (const auto i : {p, v}) {
      if (stamp <= 0.0 || state.stamp(i) < stamp) {stamp = state.stamp(i);}
    }
  }
  // Skipped (false) while a reader in another thread holds the lock: the model keeps the
  // previous state until the next update. Expected, so not logged (this runs at the control
  // rate).
  feed.model->update(feed.q, feed.v, stamp);
}

void PolicyRunner::update_kinematics() {update_kinematics(kinematics_, state_);}

PolicyPipeline::Status PolicyRunner::step(const DataStore & state, DataStore & commands)
{
  if (const bool accepted = command_gate_.open(state); accepted != commands_accepted_) {
    // Logged on transitions only, like the ones below.
    RCLCPP_WARN_EXPRESSION(logger_, !accepted, "Safety entry is 0: ignoring user commands");
    RCLCPP_INFO_EXPRESSION(logger_, accepted, "Safety entry is set: accepting user commands");
    commands_accepted_ = accepted;
  }
  if (policy_kinematics_.model != kinematics_.model) {update_kinematics(policy_kinematics_, state);}
  PolicyPipeline::Status status;
  try {
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
