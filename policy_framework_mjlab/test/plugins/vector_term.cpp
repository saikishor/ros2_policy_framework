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

// Test-only sensor term: the vector of the latest geometry_msgs/Vector3Stamped on its topic,
// stamped with its header. A TopicTerm like a real sensor term, so `max_age` is testable end
// to end.

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/interfaces/observation_term.hpp"
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
