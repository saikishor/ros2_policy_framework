// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__POLICY_NODE_HPP_
#define POLICY_FRAMEWORK__POLICY_NODE_HPP_

#include <memory>

#include "policy_framework/policy_runner.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace policy_framework
{

/**
 * @brief Standalone lifecycle node running the policies outside ros2_control.
 *
 * All inputs must come from input sources (topics) and commands leave through executors.
 * Parameters: those of PolicyRunner, plus `update_rate` (Hz, default 50), the policy step
 * rate, `autostart` (default true) and `robot_description` (the URDF, needed only by the
 * kinematics, whose joint positions and velocities must then also come from sources). The timer uses the node clock, so it follows `/clock`
 * with `use_sim_time`.
 *
 * - configure: loads everything; fails when a policy input is not provided by any source.
 * - activate: starts the timer. Every activation starts from the default (first) policy with
 *   all policies cleared, even if a non-interruptible policy was active, and clears a fault.
 *
 * Safety (see PolicyRunner::update_safety()): the checks run on every tick, before the step,
 * and the `max_age` of the step's inputs after it (PolicyRunner::check_input_ages()). A fault
 * with a latched reaction (anything but neutral_commands), including a failed policy step or a
 * stale input, stops the commands until the node is reactivated: the node does not own the
 * hardware, so it cannot hold or damp the robot.
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
  double update_rate_ = 50.0;  ///< Hz, from the parameters at configure.
  double observation_stamp_ = 0.0;  ///< Of the last successful step, for safety.max_input_age.
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__POLICY_NODE_HPP_
