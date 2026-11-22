// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/policy_node.hpp"

#include <exception>
#include <memory>
#include <string>

#include "policy_framework/policy_node_parameters.hpp"

namespace policy_framework
{

PolicyNode::PolicyNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("policy_node", options)
{
}

PolicyNode::CallbackReturn PolicyNode::on_configure(const rclcpp_lifecycle::State &)
{
  // A fresh runner on every configure so that re-configuration reloads plugins and models.
  runner_ = std::make_unique<PolicyRunner>();
  try {
    const auto params = policy_node::ParamListener(shared_from_this()).get_params();
    update_rate_ = params.update_rate;
    runner_->configure(shared_from_this(), true, params.robot_description);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  const auto missing = runner_->unprovided_state();
  if (!missing.empty()) {
    std::string list;
    for (const auto & m : missing) {list += " " + m;}
    RCLCPP_ERROR(get_logger(), "No input source provides:%s", list.c_str());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

PolicyNode::CallbackReturn PolicyNode::on_activate(const rclcpp_lifecycle::State &)
{
  runner_->restart();
  observation_stamp_ = 0.0;
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / update_rate_), [this]() {update();});
  return CallbackReturn::SUCCESS;
}

PolicyNode::CallbackReturn PolicyNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  return CallbackReturn::SUCCESS;
}

PolicyNode::CallbackReturn PolicyNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  runner_.reset();
  return CallbackReturn::SUCCESS;
}

void PolicyNode::update()
{
  const double t = now().seconds();
  runner_->read_sources(t);
  runner_->update_kinematics();
  // The node does not own the hardware: every latched reaction stops the commands, until
  // reactivated. Neutral commands keep the policy running.
  if (const auto r = runner_->update_safety(observation_stamp_, t);
    r && *r != FaultReaction::NEUTRAL_COMMANDS)
  {
    return;
  }
  const auto status = runner_->step();
  if (status == PolicyPipeline::Status::ERROR) {
    runner_->policy_failure("The policy step failed");
    return;
  }
  if (status == PolicyPipeline::Status::OK) {
    observation_stamp_ = runner_->pipeline().observation_stamp();
    runner_->write_executors();
  }
}

}  // namespace policy_framework
