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

#ifndef POLICY_FRAMEWORK__PLUGINS__EXECUTORS__MULTI_DOF_COMMAND_EXECUTOR_HPP_
#define POLICY_FRAMEWORK__PLUGINS__EXECUTORS__MULTI_DOF_COMMAND_EXECUTOR_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "control_msgs/msg/multi_dof_command.hpp"
#include "policy_framework/plugin_interfaces.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace policy_framework
{

/**
 * @brief Publishes commands as control_msgs/MultiDOFCommand, e.g. to the reference topic of
 *        pid_controller.
 *
 * Every command entry `<dof>/<values_interface>` becomes a DOF; `values` holds that entry and
 * `values_dot` holds `<dof>/<values_dot_interface>` (0 when absent or never written).
 *
 * Parameters (under `<name>.`):
 * - `topic` (default `~/<name>`)
 * - `values_interface` (default `position`)
 * - `values_dot_interface` (default `velocity`)
 */
class MultiDofCommandExecutor : public Executor
{
public:
  /// @throws std::runtime_error when no command entry matches `values_interface`.
  void configure(const PluginContext & ctx, const DataStore & commands) override;
  void write(const DataStore & commands) override;

private:
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::MultiDOFCommand>> rt_pub_;
  control_msgs::msg::MultiDOFCommand msg_;
  std::vector<size_t> values_idx_;
  std::vector<std::optional<size_t>> values_dot_idx_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__PLUGINS__EXECUTORS__MULTI_DOF_COMMAND_EXECUTOR_HPP_
