// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework_mjlab/mjlab_postprocessor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "policy_framework_mjlab/mjlab_postprocessor_parameters.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "policy_framework_mjlab/metadata.hpp"

using policy_framework::DataStore;

namespace policy_framework_mjlab
{

class GainsSender
{
public:
  GainsSender(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & target)
  : target_(target), logger_(node->get_logger()),
    client_(std::make_shared<rclcpp::AsyncParametersClient>(node, target)),
    timer_(node->create_wall_timer(std::chrono::milliseconds(500), [this]() {send();}))
  {
  }

  /// Real-time safe: replaces any request not sent yet.
  void request(const GainsRequest * r) {pending_.store(r);}

  /// Drop @p r if still pending; once it returns, @p r is no longer read (its owner may go).
  void cancel(const GainsRequest * r)
  {
    const std::lock_guard<std::mutex> lock(taking_);
    pending_.compare_exchange_strong(r, nullptr);
  }

  /// Shared sender for (@p node, @p target): one timer and one latest-wins slot per target.
  static std::shared_ptr<GainsSender> get(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & target)
  {
    static std::mutex mutex;
    static std::map<std::string, std::weak_ptr<GainsSender>> senders;
    const std::lock_guard<std::mutex> lock(mutex);
    auto & weak = senders[std::string(node->get_node_base_interface()->get_fully_qualified_name()) + "|" + target];
    auto sender = weak.lock();
    if (!sender) {
      sender = std::make_shared<GainsSender>(node, target);
      weak = sender;
    }
    return sender;
  }

private:
  void send()
  {
    if (!pending_.load()) {return;}
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 5000, "Waiting for %s parameter services to set the gains",
        target_.c_str());
      return;
    }
    std::vector<rclcpp::Parameter> gains;
    std::string policy;
    {
      // With cancel(): the request's owner cannot destroy it while it is copied.
      const std::lock_guard<std::mutex> lock(taking_);
      const auto * r = pending_.exchange(nullptr);
      if (!r) {return;}
      gains = r->gains;
      policy = r->policy;
    }
    client_->set_parameters(
      gains,
      [this, gains, policy](
        std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
        const auto results = f.get();
        size_t failed = 0;
        for (size_t i = 0; i < results.size(); ++i) {
          if (!results[i].successful) {
            ++failed;
            RCLCPP_ERROR(
              logger_, "[%s] Failed to set %s: %s", policy.c_str(), gains[i].get_name().c_str(),
              results[i].reason.c_str());
          }
        }
        RCLCPP_INFO(
          logger_, "[%s] Sent %zu gains to %s (%zu failed)", policy.c_str(), results.size(),
          target_.c_str(), failed);
      });
  }

  std::string target_;
  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::AsyncParametersClient::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<const GainsRequest *> pending_{nullptr};
  std::mutex taking_;  ///< Non real-time: send() and cancel() only, never request().
};

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
  const auto & md = model.metadata;
  const auto [joints, defaults] = policy_framework_mjlab::joints(md);
  scale_ = parse_doubles(require(md, "action_scale"));
  const auto params = mjlab_postprocessor::ParamListener(ctx.node, ctx.name).get_params();
  const auto & excluded = params.excluded_joints;

  std::vector<std::string> actuated;
  offset_.clear();
  for (size_t i = 0; i < joints.size(); ++i) {
    if (std::find(excluded.begin(), excluded.end(), joints[i]) == excluded.end()) {
      actuated.push_back(joints[i]);
      offset_.push_back(defaults[i]);
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
    position_idx_.push_back(commands.add(j + "/" + params.position_interface));
    if (!params.velocity_interface.empty()) {
      velocity_idx_.push_back(commands.add(j + "/" + params.velocity_interface));
    }
  }
  const auto stiffness = md.count("joint_stiffness") ?
    parse_doubles(md.at("joint_stiffness")) : std::vector<double>{};
  const auto damping = md.count("joint_damping") ?
    parse_doubles(md.at("joint_damping")) : std::vector<double>{};
  std::vector<rclcpp::Parameter> gain_parameters;
  std::string gains_error;  // why the metadata gains are unusable, empty when they are fine
  try {
    gain_parameters = make_gain_parameters(actuated, stiffness, damping);
  } catch (const std::invalid_argument & e) {
    gains_error = e.what();
  }
  gain_idx_.clear();
  gain_values_.clear();
  if (params.gains_as_commands) {
    if (!gains_error.empty()) {
      throw std::runtime_error("gains_as_commands: model metadata gains: " + gains_error);
    }
    for (size_t i = 0; i < actuated.size(); ++i) {
      gain_idx_.push_back(commands.add(actuated[i] + "/stiffness"));
      gain_values_.push_back(stiffness[i]);
      gain_idx_.push_back(commands.add(actuated[i] + "/damping"));
      gain_values_.push_back(damping[i]);
    }
  }

  if (gains_sender_) {gains_sender_->cancel(&gains_request_);}
  gains_sender_.reset();
  gains_request_ = GainsRequest{ctx.name, gain_parameters};
  if (params.gains_controller.empty()) {
    return;
  }
  if (!gains_error.empty()) {
    // Same as the reference deployer: run the policy, leave the controller gains untouched.
    RCLCPP_ERROR(
      ctx.node->get_logger().get_child(ctx.name), "Not sending gains to %s: %s",
      params.gains_controller.c_str(), gains_error.c_str());
    return;
  }
  gains_sender_ = GainsSender::get(ctx.node, params.gains_controller);
}

MjlabPostprocessor::~MjlabPostprocessor()
{
  // The sender may outlive this policy (shared with others): never leave it our request.
  if (gains_sender_) {gains_sender_->cancel(&gains_request_);}
}

void MjlabPostprocessor::reset()
{
  if (gains_sender_) {gains_sender_->request(&gains_request_);}
}

void MjlabPostprocessor::initial_commands(DataStore & commands)
{
  for (size_t i = 0; i < gain_idx_.size(); ++i) {commands.set(gain_idx_[i], gain_values_[i], 0.0);}
}

bool MjlabPostprocessor::compute(
  const policy_framework::TensorMap & outputs, double stamp, DataStore & commands)
{
  const auto & a = outputs.at("actions").data;
  for (size_t i = 0; i < position_idx_.size(); ++i) {
    commands.set(position_idx_[i], scale_[i] * a[i] + offset_[i], stamp);
  }
  for (const auto i : velocity_idx_) {commands.set(i, 0.0, stamp);}
  for (size_t i = 0; i < gain_idx_.size(); ++i) {commands.set(gain_idx_[i], gain_values_[i], stamp);}
  return true;
}

}  // namespace policy_framework_mjlab

PLUGINLIB_EXPORT_CLASS(policy_framework_mjlab::MjlabPostprocessor, policy_framework::Postprocessor)
