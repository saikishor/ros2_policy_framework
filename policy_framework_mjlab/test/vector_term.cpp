// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

// Test-only sensor term: the vector of the latest geometry_msgs/Vector3Stamped on its topic,
// stamped with its header. A TopicTerm like a real sensor term, so `max_age` is testable end
// to end.

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/observation_term.hpp"
#include "rclcpp/time.hpp"

namespace policy_framework_mjlab_test
{

class VectorTerm : public policy_framework::TopicTerm<geometry_msgs::msg::Vector3Stamped>
{
public:
  VectorTerm() : TopicTerm("vector") {}

protected:
  size_t configure_term(
    const policy_framework::PluginContext &, const policy_framework::ModelInfo &) override
  {
    return 3;
  }

  double compute(
    const policy_framework::DataStore &, const policy_framework::TensorMap &,
    float * out) override
  {
    const auto & msg = *latest().msg;
    out[0] = static_cast<float>(msg.vector.x);
    out[1] = static_cast<float>(msg.vector.y);
    out[2] = static_cast<float>(msg.vector.z);
    return rclcpp::Time(msg.header.stamp).seconds();
  }
};

}  // namespace policy_framework_mjlab_test

PLUGINLIB_EXPORT_CLASS(
  policy_framework_mjlab_test::VectorTerm, policy_framework::ObservationTerm)
