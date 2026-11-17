// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/hosts/policy_replay.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace policy_framework
{

StepReplayer::Summary replay_bag(
  const std::string & bag, const std::string & topic, std::shared_ptr<Inference> inference,
  const std::string & policy)
{
  const std::string type = rosidl_generator_traits::name<msg::PolicyStep>();
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  std::vector<std::string> candidates;
  for (const auto & t : reader.get_all_topics_and_types()) {
    if (!topic.empty() && t.name == topic && t.type != type) {
      throw std::runtime_error("Topic '" + topic + "' of the bag is a " + t.type + ", not a " + type);
    }
    if (t.type == type && (topic.empty() || t.name == topic)) {candidates.push_back(t.name);}
  }
  if (candidates.empty()) {
    throw std::runtime_error(
            "The bag has no " + type + " topic" + (topic.empty() ? "" : " named '" + topic + "'") +
            " (record ~/policy_step of a runner with publish_steps: true)");
  }
  if (candidates.size() > 1) {
    std::string list;
    for (const auto & c : candidates) {list += " " + c;}
    throw std::runtime_error("The bag has several " + type + " topics, name one:" + list);
  }
  rosbag2_storage::StorageFilter filter;
  filter.topics = {candidates.front()};
  reader.set_filter(filter);

  StepReplayer replayer(std::move(inference), policy);
  rclcpp::Serialization<msg::PolicyStep> serialization;
  msg::PolicyStep step;
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    const rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
    serialization.deserialize_message(&serialized, &step);
    replayer.replay(step);
  }
  return replayer.summary();
}

}  // namespace policy_framework
