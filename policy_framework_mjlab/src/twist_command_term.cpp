// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework_mjlab/twist_command_term.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace policy_framework_mjlab
{

size_t TwistCommandTerm::configure_term(
  const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo &)
{
  timeout_ = policy_framework::get_parameter<double>(ctx, "timeout", 1.0);
  return 3;
}

void TwistCommandTerm::compute(
  const policy_framework::DataStore &, const policy_framework::TensorMap &, float * out)
{
  const auto in = latest();
  if (!in.msg || (timeout_ > 0.0 && now() - in.receive_time > timeout_)) {
    out[0] = out[1] = out[2] = 0.0F;
    return;
  }
  out[0] = static_cast<float>(in.msg->linear.x);
  out[1] = static_cast<float>(in.msg->linear.y);
  out[2] = static_cast<float>(in.msg->angular.z);
}

}  // namespace policy_framework_mjlab

PLUGINLIB_EXPORT_CLASS(policy_framework_mjlab::TwistCommandTerm, policy_framework::ObservationTerm)
