// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__POLICY_NODE_HPP_
#define POLICY_FRAMEWORK__POLICY_NODE_HPP_

#include "policy_framework/policy_runner.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace policy_framework
{

/**
 * @brief Standalone lifecycle node running the policies outside ros2_control.
 *
 * All inputs must come from input sources (topics) and commands leave through executors.
 * Parameters: those of PolicyRunner, plus `update_rate` (Hz, default 50), the policy step
 * rate. The timer uses the node clock, so it follows `/clock` with `use_sim_time`.
 *
 * - configure: loads everything; fails when a policy input is not provided by any source.
 * - activate: starts the timer. Every activation starts from the default (first) policy with
 *   all policies cleared, even if a non-interruptible policy was active.
 * - deactivate: stops the timer.
 */
class PolicyNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PolicyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

private:
  void update();

  std::unique_ptr<PolicyRunner> runner_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__POLICY_NODE_HPP_
