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

#ifndef POLICY_FRAMEWORK_MJLAB__TWIST_COMMAND_TERM_HPP_
#define POLICY_FRAMEWORK_MJLAB__TWIST_COMMAND_TERM_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "policy_framework/interfaces/observation_term.hpp"

namespace policy_framework_mjlab
{

/**
 * @brief mjlab velocity `command` term (linear.x, linear.y, angular.z) read directly from a
 *        geometry_msgs/Twist topic, without going through the state store.
 *
 * A policy_framework::CommandTerm: parameters (under `<policy>.terms.<term>.`) `topic`
 * (default `cmd_vel`) and `timeout` (seconds, default 1.0, above zero); zero velocity is its
 * neutral command (no command, an old one, a closed CommandGate). Stamped now: a Twist has no
 * header.
 */
class TwistCommandTerm : public policy_framework::CommandTerm<geometry_msgs::msg::Twist>
{
public:
  TwistCommandTerm() : CommandTerm("cmd_vel", 1.0) {}

protected:
  size_t configure_command(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model)
  override;
  void write_command(
    const geometry_msgs::msg::Twist & msg, const policy_framework::DataStore & state,
    float * out) override;
  void write_neutral(const policy_framework::DataStore & state, float * out) override;
};

}  // namespace policy_framework_mjlab

#endif  // POLICY_FRAMEWORK_MJLAB__TWIST_COMMAND_TERM_HPP_
