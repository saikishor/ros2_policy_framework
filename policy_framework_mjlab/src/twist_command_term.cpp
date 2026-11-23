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
