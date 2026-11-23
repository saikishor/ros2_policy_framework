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

// End to end: a sensor term's `max_age` through the MjlabPreprocessor stamps into the
// PolicyRunner input age check that both hosts call after each successful step.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "policy_framework/hosts/policy_runner.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::FaultReaction;
using policy_framework::PolicyPipeline;
using policy_framework::PolicyRunner;
using StringArray = std::vector<std::string>;

namespace
{

class TermMaxAgeTest : public ::testing::Test
{
protected:
  // One policy whose only observation is the test VectorTerm on @p topic.
  void configure(const std::string & topic, std::vector<rclcpp::Parameter> extra = {})
  {
    std::vector<rclcpp::Parameter> params = {
      {"policies", StringArray{"walk"}},
      {"walk.preprocessor", "policy_framework_mjlab/MjlabPreprocessor"},
      {"walk.model_path", std::string(TEST_MODEL)},
      {"walk.terms.sensor.type", "policy_framework_mjlab_test/VectorTerm"},
      {"walk.terms.sensor.topic", topic},
      {"walk.postprocessor", "policy_framework/SplitPostprocessor"},
      {"walk.output_entries", StringArray{"a/position"}},
    };
    params.insert(params.end(), extra.begin(), extra.end());
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(params);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("term_max_age", opts);
    helper = std::make_shared<rclcpp::Node>("term_max_age_helper");
    exec.add_node(node->get_node_base_interface());
    exec.add_node(helper);
    runner.configure(node, false);
    pub = helper->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic, 10);
  }

  // Publish one message stamped now; it arrived between `sent` and the returned time (node
  // clock, seconds).
  double publish()
  {
    for (int i = 0; i < 100 && pub->get_subscription_count() == 0; ++i) {spin_for(10);}
    sent = node->now().seconds();
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = helper->now();
    msg.vector.x = 0.7;
    pub->publish(msg);
    spin_for(100);
    return node->now().seconds();
  }

  void spin_for(int ms)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr helper;
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub;
  PolicyRunner runner;
  double sent = 0.0;
};

}  // namespace

TEST_F(TermMaxAgeTest, AStaleSensorTermFaultsWithItsReactionAfterTheStep)
{
  configure("/term_max_age", {{"walk.terms.sensor.max_age", 0.5},
    {"walk.terms.sensor.max_age_on_fault", "damp"}});
  const auto reactions = runner.fault_reactions();
  EXPECT_NE(std::find(reactions.begin(), reactions.end(), FaultReaction::DAMP), reactions.end());

  runner.read_sources(node->now().seconds());
  ASSERT_EQ(runner.step(), PolicyPipeline::Status::WAITING_FOR_DATA);  // no message yet

  const double received = publish();
  runner.read_sources(received);
  ASSERT_EQ(runner.step(), PolicyPipeline::Status::OK);
  EXPECT_DOUBLE_EQ(runner.commands().get(runner.commands().index("a/position")), 0.7F);
  EXPECT_FALSE(runner.check_input_ages(sent + 0.4).has_value());  // younger than 0.5 s
  EXPECT_EQ(runner.check_input_ages(received + 0.6), FaultReaction::DAMP);
}

TEST_F(TermMaxAgeTest, ASensorTermWithoutMaxAgeIsNotChecked)
{
  configure("/term_no_max_age");
  const double received = publish();
  runner.read_sources(received);
  ASSERT_EQ(runner.step(), PolicyPipeline::Status::OK);
  EXPECT_FALSE(runner.check_input_ages(received + 100.0).has_value());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
