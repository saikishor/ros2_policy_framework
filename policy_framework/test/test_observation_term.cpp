// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "policy_framework/observation_term.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::DataStore;
using policy_framework::ModelInfo;
using policy_framework::PluginContext;
using policy_framework::TensorMap;
using Twist = geometry_msgs::msg::Twist;

namespace
{

/// Minimal topic term: the linear x of the last Twist.
class LinearXTerm : public policy_framework::TopicTerm<Twist>
{
public:
  LinearXTerm() : TopicTerm("~/default_topic") {}
  void compute(const DataStore &, const TensorMap &, float * out) override
  {
    out[0] = static_cast<float>(latest().msg->linear.x);
  }

protected:
  size_t configure_term(const PluginContext &, const ModelInfo &) override {return 1;}
};

}  // namespace

TEST(TopicTerm, IsReadyOnceAMessageArrivedOnItsTopicParameter)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"p.terms.x.topic", "/test_topic_term"}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_topic_term", opts);
  DataStore state;
  LinearXTerm term;
  EXPECT_EQ(term.configure({node, "p.terms.x"}, ModelInfo{}, state), 1U);
  EXPECT_EQ(state.size(), 0U);  // a topic term does not go through the state store
  EXPECT_FALSE(term.ready(state));

  auto pub_node = std::make_shared<rclcpp::Node>("test_topic_term_pub");
  auto pub = pub_node->create_publisher<Twist>("/test_topic_term", 10);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  Twist msg;
  msg.linear.x = 0.25;
  for (int i = 0; i < 500 && !term.ready(state); ++i) {
    pub->publish(msg);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(term.ready(state));
  float out = 0.0F;
  term.compute(state, TensorMap{}, &out);
  EXPECT_FLOAT_EQ(out, 0.25F);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
