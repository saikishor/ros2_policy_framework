// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "policy_framework/policy_node.hpp"
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
