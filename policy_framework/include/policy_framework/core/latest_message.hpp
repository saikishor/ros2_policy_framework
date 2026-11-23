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

#ifndef POLICY_FRAMEWORK__CORE__LATEST_MESSAGE_HPP_
#define POLICY_FRAMEWORK__CORE__LATEST_MESSAGE_HPP_

#include <memory>
#include <string>

#include "realtime_tools/realtime_buffer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace policy_framework
{

struct PluginContext;

/**
 * @brief The `<ctx.name>.topic` parameter (declared and validated once), or @p default_topic
 *        when it is empty. Shared by the topic input sources and topic observation terms.
 */
std::string topic_parameter(const PluginContext & ctx, const std::string & default_topic);

/**
 * @brief Subscription that hands the latest message over to the control cycle, real-time safe.
 *
 * Shared by the topic input sources and the topic observation terms. Not copyable or movable:
 * the subscription callback refers to this object.
 */
template<typename MsgT>
class LatestMessage
{
public:
  struct Received
  {
    std::shared_ptr<const MsgT> msg;  ///< Null until the first message arrives.
    double receive_time = 0.0;        ///< Seconds, node clock.
  };

  LatestMessage() = default;
  LatestMessage(const LatestMessage &) = delete;
  LatestMessage & operator=(const LatestMessage &) = delete;

  /// Subscribe to @p topic on @p node (replacing any previous subscription).
  void subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & topic)
  {
    clock_ = node->get_clock();
    sub_ = node->template create_subscription<MsgT>(
      topic, rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<const MsgT> msg) {
        buffer_.writeFromNonRT(Received{msg, clock_->now().seconds()});
      });
  }

  /// @return The latest message and when it arrived. Real-time safe.
  Received get() {return *buffer_.readFromRT();}

  /// @return Current time of the node clock (the receive_time clock), seconds.
  double now() const {return clock_->now().seconds();}

private:
  rclcpp::Clock::SharedPtr clock_;
  typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
  realtime_tools::RealtimeBuffer<Received> buffer_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__CORE__LATEST_MESSAGE_HPP_
