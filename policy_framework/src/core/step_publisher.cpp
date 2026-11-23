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

#include "policy_framework/core/step_publisher.hpp"

#include <memory>
#include <vector>

namespace policy_framework
{
namespace
{

void fill_tensors(const TensorMap & tensors, std::vector<msg::Tensor> & out)
{
  out.resize(tensors.size());
  size_t i = 0;
  for (const auto & [name, tensor] : tensors) {
    out[i].name = name;
    out[i].shape = tensor.shape;
    out[i].data = tensor.data;
    ++i;
  }
}

uint8_t to_msg(PolicyPipeline::Status status)
{
  switch (status) {
    case PolicyPipeline::Status::OK: return msg::PolicyStep::STATUS_OK;
    case PolicyPipeline::Status::WAITING_FOR_DATA: return msg::PolicyStep::STATUS_WAITING_FOR_DATA;
    default: return msg::PolicyStep::STATUS_ERROR;
  }
}

}  // namespace

void fill_policy_step(
  const PolicyPipeline & pipeline, PolicyPipeline::Status status, const DataStore & commands,
  double step_time_us, msg::PolicyStep & msg)
{
  // Assignments reuse the capacity of the previous message: no allocation while the layout
  // (policy, tensor sizes) stays the same.
  msg.policy = pipeline.active_policy();
  msg.step = pipeline.steps();
  msg.status = to_msg(status);
  msg.step_time_us = step_time_us;
  msg.observation_stamp = pipeline.observation_stamp();
  const auto & stamps = pipeline.input_stamps();
  msg.input_names.resize(stamps.size());
  msg.input_stamps.resize(stamps.size());
  for (size_t i = 0; i < stamps.size(); ++i) {
    msg.input_names[i] = stamps[i].name;
    msg.input_stamps[i] = stamps[i].stamp;
  }
  fill_tensors(pipeline.inputs(), msg.inputs);
  fill_tensors(pipeline.outputs(), msg.outputs);
  msg.command_names = commands.names();
  msg.commands.resize(commands.size());
  for (size_t i = 0; i < commands.size(); ++i) {msg.commands[i] = commands.get(i);}
}

StepPublisher::StepPublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
: clock_(node->get_clock())
{
  // A plain publisher: a lifecycle one drops everything while the node is not active.
  publisher_ = std::make_unique<realtime_tools::RealtimePublisher<msg::PolicyStep>>(
    rclcpp::create_publisher<msg::PolicyStep>(
      node->get_node_topics_interface(), "~/policy_step", rclcpp::SystemDefaultsQoS()));
}

void StepPublisher::publish(
  const PolicyPipeline & pipeline, PolicyPipeline::Status status, const DataStore & commands,
  double step_time_us)
{
  msg_.header.stamp = clock_->now();
  fill_policy_step(pipeline, status, commands, step_time_us, msg_);
  // Drops the message if the publishing thread is still busy with the previous one.
  publisher_->try_publish(msg_);
}

}  // namespace policy_framework
