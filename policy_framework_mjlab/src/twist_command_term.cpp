// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework_mjlab/twist_command_term.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace policy_framework_mjlab
{

size_t TwistCommandTerm::configure_command(
  const policy_framework::PluginContext &, const policy_framework::ModelInfo &)
{
  return 3;
}

void TwistCommandTerm::write_command(
  const geometry_msgs::msg::Twist & msg, const policy_framework::DataStore &, float * out)
{
  out[0] = static_cast<float>(msg.linear.x);
  out[1] = static_cast<float>(msg.linear.y);
  out[2] = static_cast<float>(msg.angular.z);
}

void TwistCommandTerm::write_neutral(const policy_framework::DataStore &, float * out)
{
  out[0] = out[1] = out[2] = 0.0F;
}

}  // namespace policy_framework_mjlab

PLUGINLIB_EXPORT_CLASS(policy_framework_mjlab::TwistCommandTerm, policy_framework::ObservationTerm)
