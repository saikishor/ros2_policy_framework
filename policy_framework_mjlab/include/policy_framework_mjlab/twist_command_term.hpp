// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK_MJLAB__TWIST_COMMAND_TERM_HPP_
#define POLICY_FRAMEWORK_MJLAB__TWIST_COMMAND_TERM_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "policy_framework/observation_term.hpp"

namespace policy_framework_mjlab
{

/**
 * @brief mjlab velocity `command` term (linear.x, linear.y, angular.z) read directly from a
 *        geometry_msgs/Twist topic, without going through the state store.
 *
 * Parameters (under `<policy>.terms.<term>.`): `topic` (default `cmd_vel`), `timeout` (seconds,
 * default 1.0, <= 0 disables it). Always ready: a command that never arrived, is older than
 * `timeout` or arrives while the host's `safety_entry` is 0 (policy_framework::CommandGate) is
 * zero, so a policy neither waits for a command nor acts on a stale or unsafe one.
 *
 * Stamp: the receive time (a Twist has no header); the current time for a zero command.
 */
class TwistCommandTerm : public policy_framework::TopicTerm<geometry_msgs::msg::Twist>
{
public:
  TwistCommandTerm() : TopicTerm("cmd_vel") {}
  bool ready(const policy_framework::DataStore &) override {return true;}
  double compute(
    const policy_framework::DataStore & state, const policy_framework::TensorMap & last_outputs,
    float * out) override;

protected:
  size_t configure_term(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model)
  override;

private:
  policy_framework::CommandGate gate_;
  double timeout_ = 1.0;
};

}  // namespace policy_framework_mjlab

#endif  // POLICY_FRAMEWORK_MJLAB__TWIST_COMMAND_TERM_HPP_
