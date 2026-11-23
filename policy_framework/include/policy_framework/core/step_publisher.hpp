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

#ifndef POLICY_FRAMEWORK__CORE__STEP_PUBLISHER_HPP_
#define POLICY_FRAMEWORK__CORE__STEP_PUBLISHER_HPP_

#include <memory>

#include "policy_framework/core/data_store.hpp"
#include "policy_framework/msg/policy_step.hpp"
#include "policy_framework/core/policy_pipeline.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace policy_framework
{

/**
 * @brief Fill @p msg with the last step of @p pipeline: the active policy, its model inputs and
 *        outputs, the stamps of its inputs and every entry of @p commands.
 *
 * Call it from the thread that ran the step. It allocates only when the layout changes, that
 * is on the first step and when the active policy changed to one with other tensor sizes.
 * The header is left to the caller.
 */
void fill_policy_step(
  const PolicyPipeline & pipeline, PolicyPipeline::Status status, const DataStore & commands,
  double step_time_us, msg::PolicyStep & msg);

/**
 * @brief Publishes every policy step as a policy_framework/msg/PolicyStep on `~/policy_step`,
 *        to record what the model saw and answered (e.g. `ros2 bag record`) and to run it
 *        through a model again offline (policy_replay).
 *
 * publish() is called from the control cycle: the message is handed to a non real-time
 * publishing thread, and dropped when that thread is still busy with the previous one.
 * The publisher is not a lifecycle publisher, so steps are also published while the host is
 * not active (e.g. in tests driving a runner directly).
 */
class StepPublisher
{
public:
  explicit StepPublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /// Publish the last step of @p pipeline, see fill_policy_step().
  void publish(
    const PolicyPipeline & pipeline, PolicyPipeline::Status status, const DataStore & commands,
    double step_time_us);

private:
  rclcpp::Clock::SharedPtr clock_;
  std::unique_ptr<realtime_tools::RealtimePublisher<msg::PolicyStep>> publisher_;
  msg::PolicyStep msg_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__CORE__STEP_PUBLISHER_HPP_
