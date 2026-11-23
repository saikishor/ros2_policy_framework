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

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "policy_framework/hosts/policy_node.hpp"
#include "policy_framework/policy_node_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

/// Runs a PolicyNode. With the `autostart` parameter (default true) it configures and
/// activates itself, so it can be used like a regular node; otherwise drive it with
/// `ros2 lifecycle set`.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<policy_framework::PolicyNode>();
  if (policy_node::ParamListener(node).get_params().autostart) {
    if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
      node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_FATAL(node->get_logger(), "Failed to start, exiting");
      rclcpp::shutdown();
      return 1;
    }
  }
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
