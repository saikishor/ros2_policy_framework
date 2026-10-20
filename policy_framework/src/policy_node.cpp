// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/policy_node.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include "policy_node_parameters.hpp"

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
    update_rate_ = policy_node::ParamListener(shared_from_this()).get_params().update_rate;
    runner_->configure(shared_from_this());
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
  runner_->pipeline().restart();
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
  runner_->read_sources(now().seconds());
  if (runner_->step() == PolicyPipeline::Status::OK) {
    runner_->write_executors();
  }
}

}  // namespace policy_framework
