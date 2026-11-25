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

#include "policy_framework/plugins/executors/multi_dof_command_executor.hpp"

#include "core/entry_name.hpp"

#include <stdexcept>
#include <string>
#include <fmt/format.h>

#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/multi_dof_command_executor_parameters.hpp"

namespace policy_framework
{

void MultiDofCommandExecutor::configure(const PluginContext & ctx, const DataStore & commands)
{
  const auto params = multi_dof_command_executor::ParamListener(ctx.node, ctx.name).get_params();
  const auto topic = params.topic.empty() ? "~/" + ctx.name : params.topic;
  const auto values_suffix = "/" + params.values_interface;
  const auto & dot_iface = params.values_dot_interface;

  msg_ = control_msgs::msg::MultiDOFCommand();
  values_idx_.clear();
  values_dot_idx_.clear();
  for (size_t i = 0; i < commands.size(); ++i) {
    const auto dof = strip_suffix(commands.names()[i], values_suffix);
    if (!dof) {continue;}
    msg_.dof_names.push_back(*dof);
    values_idx_.push_back(i);
    values_dot_idx_.push_back(
      dot_iface.empty() ? std::nullopt : commands.find(*dof + "/" + dot_iface));
  }
  if (values_idx_.empty()) {
    throw std::runtime_error(
            fmt::format(
              "Executor '{}': no command entry ends with '{}'", ctx.name, values_suffix));
  }
  msg_.values.assign(values_idx_.size(), 0.0);
  msg_.values_dot.assign(values_idx_.size(), 0.0);
  rt_pub_ = std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::MultiDOFCommand>>(
    ctx.node->create_publisher<control_msgs::msg::MultiDOFCommand>(
      topic, rclcpp::SystemDefaultsQoS()));
}

void MultiDofCommandExecutor::write(const DataStore & commands)
{
  for (size_t i = 0; i < values_idx_.size(); ++i) {
    msg_.values[i] = commands.get(values_idx_[i]);
    msg_.values_dot[i] = values_dot_idx_[i] ? commands.get(*values_dot_idx_[i]) : 0.0;
  }
  // Drops the message if the publishing thread is still busy with the previous one.
  rt_pub_->try_publish(msg_);
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::MultiDofCommandExecutor, policy_framework::Executor)
