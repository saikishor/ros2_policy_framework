// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework_mjlab/mjlab_postprocessor.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "policy_framework_mjlab/metadata.hpp"

using policy_framework::DataStore;
using policy_framework::get_parameter;

namespace policy_framework_mjlab
{

std::vector<rclcpp::Parameter> make_gain_parameters(
  const std::vector<std::string> & joints, const std::vector<double> & stiffness,
  const std::vector<double> & damping)
{
  if (joints.empty() || stiffness.size() != joints.size() || damping.size() != joints.size()) {
    throw std::invalid_argument(
            "Expected one stiffness and damping per actuated joint (" +
            std::to_string(joints.size()) + "), got " + std::to_string(stiffness.size()) +
            " and " + std::to_string(damping.size()));
  }
  std::vector<rclcpp::Parameter> params;
  for (size_t i = 0; i < joints.size(); ++i) {
    params.emplace_back("gains." + joints[i] + ".p", stiffness[i]);
    params.emplace_back("gains." + joints[i] + ".d", damping[i]);
  }
  return params;
}

void MjlabPostprocessor::configure(
  const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
  DataStore & commands)
{
  logger_ = ctx.node->get_logger().get_child(ctx.name);
  const auto & md = model.metadata;
  const auto joints = split(require(md, "joint_names"));
  const auto defaults = parse_doubles(require(md, "default_joint_pos"));
  scale_ = parse_doubles(require(md, "action_scale"));
  const auto excluded = get_parameter<std::vector<std::string>>(ctx, "excluded_joints", {});
  const auto pos_iface = get_parameter<std::string>(ctx, "position_interface", "position");
  const auto vel_iface = get_parameter<std::string>(ctx, "velocity_interface", "velocity");
  gains_controller_ = get_parameter<std::string>(ctx, "gains_controller", "");

  std::vector<std::string> actuated;
  offset_.clear();
  for (size_t i = 0; i < joints.size(); ++i) {
    if (std::find(excluded.begin(), excluded.end(), joints[i]) == excluded.end()) {
      actuated.push_back(joints[i]);
      offset_.push_back(defaults.at(i));
    }
  }
  const auto actions = model.outputs.find("actions");
  if (actions == model.outputs.end()) {
    throw std::runtime_error("Model has no 'actions' output");
  }
  if (actuated.size() != actions->second.data.size() || scale_.size() != actuated.size()) {
    throw std::runtime_error(
            "Model has " + std::to_string(actions->second.data.size()) + " actions and " +
            std::to_string(scale_.size()) + " action scales but " +
            std::to_string(actuated.size()) +
            " actuated joints; set 'excluded_joints' to the joints the policy does not drive");
  }
  position_idx_.clear();
  velocity_idx_.clear();
  for (const auto & j : actuated) {
    position_idx_.push_back(commands.add(j + "/" + pos_iface));
    if (!vel_iface.empty()) {velocity_idx_.push_back(commands.add(j + "/" + vel_iface));}
  }

  gains_.clear();
  gains_client_.reset();
  gains_timer_.reset();
  if (gains_controller_.empty()) {
    return;
  }
  const auto stiffness = md.count("joint_stiffness") ?
    parse_doubles(md.at("joint_stiffness")) : std::vector<double>{};
  const auto damping = md.count("joint_damping") ?
    parse_doubles(md.at("joint_damping")) : std::vector<double>{};
  try {
    gains_ = make_gain_parameters(actuated, stiffness, damping);
  } catch (const std::invalid_argument & e) {
    // Same as the reference deployer: run the policy, leave the controller gains untouched.
    RCLCPP_ERROR(logger_, "Not sending gains to %s: %s", gains_controller_.c_str(), e.what());
    return;
  }
  gains_client_ = std::make_shared<rclcpp::AsyncParametersClient>(ctx.node, gains_controller_);
  gains_timer_ = ctx.node->create_wall_timer(
    std::chrono::milliseconds(500), [this]() {send_gains();});
}

void MjlabPostprocessor::reset()
{
  gains_pending_.store(gains_client_ != nullptr);
}

void MjlabPostprocessor::send_gains()
{
  if (!gains_pending_.load()) {return;}
  if (!gains_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(RCL_STEADY_TIME), 5000,
      "Waiting for %s parameter services to set the gains", gains_controller_.c_str());
    return;
  }
  gains_pending_.store(false);
  gains_client_->set_parameters(
    gains_, [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
      const auto results = f.get();
      size_t failed = 0;
      for (size_t i = 0; i < results.size(); ++i) {
        if (!results[i].successful) {
          ++failed;
          RCLCPP_ERROR(
            logger_, "Failed to set %s: %s", gains_[i].get_name().c_str(),
            results[i].reason.c_str());
        }
      }
      RCLCPP_INFO(
        logger_, "Sent %zu gains to %s (%zu failed)", results.size(), gains_controller_.c_str(),
        failed);
    });
}

bool MjlabPostprocessor::compute(
  const policy_framework::TensorMap & outputs, DataStore & commands)
{
  const auto & a = outputs.at("actions").data;
  for (size_t i = 0; i < position_idx_.size(); ++i) {
    commands.set(position_idx_[i], scale_[i] * a[i] + offset_[i], 0.0);
  }
  for (const auto i : velocity_idx_) {commands.set(i, 0.0, 0.0);}
  return true;
}

}  // namespace policy_framework_mjlab

PLUGINLIB_EXPORT_CLASS(policy_framework_mjlab::MjlabPostprocessor, policy_framework::Postprocessor)
