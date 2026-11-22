// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "policy_framework/observation_term.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::DataStore;
using policy_framework::ModelInfo;
using policy_framework::PluginContext;
using policy_framework::TensorMap;
using Twist = geometry_msgs::msg::Twist;
using TwistStamped = geometry_msgs::msg::TwistStamped;

namespace
{

/// Minimal topic term: the linear x of the last Twist.
class LinearXTerm : public policy_framework::TopicTerm<Twist>
{
public:
  LinearXTerm() : TopicTerm("~/default_topic") {}
  double compute(const DataStore &, const TensorMap &, float * out) override
  {
    const auto in = latest();
    out[0] = static_cast<float>(in.msg->linear.x);
    return in.receive_time;
  }

protected:
  size_t configure_term(const PluginContext &, const ModelInfo &) override {return 1;}
};

/// Minimal command terms: the linear x of the last command, 0 as the neutral command.
class VxCommand : public policy_framework::CommandTerm<Twist>
{
public:
  VxCommand() : CommandTerm("~/cmd", 0.3) {}

protected:
  size_t configure_command(const PluginContext &, const ModelInfo &) override {return 1;}
  void write_command(const Twist & msg, const DataStore &, float * out) override
  {
    out[0] = static_cast<float>(msg.linear.x);
  }
  void write_neutral(const DataStore &, float * out) override {out[0] = 0.0F;}
};

class StampedVxCommand : public policy_framework::CommandTerm<TwistStamped>
{
public:
  StampedVxCommand() : CommandTerm("~/cmd", 0.3) {}

protected:
  size_t configure_command(const PluginContext &, const ModelInfo &) override {return 1;}
  void write_command(const TwistStamped & msg, const DataStore &, float * out) override
  {
    out[0] = static_cast<float>(msg.twist.linear.x);
  }
  void write_neutral(const DataStore &, float * out) override {out[0] = 0.0F;}
};

/// A node with the parameters of the command term "p.terms.cmd" on @p topic.
rclcpp_lifecycle::LifecycleNode::SharedPtr command_node(
  const std::string & name, const std::string & topic, double timeout = 0.3)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"p.terms.cmd.topic", topic}, {"p.terms.cmd.timeout", timeout}});
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, opts);
}

/// Publish @p msg on @p topic until @p done.
template<typename Msg, typename Done>
void publish_until(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & topic,
  const Msg & msg, Done done)
{
  auto pub_node = std::make_shared<rclcpp::Node>(node->get_name() + std::string("_pub"));
  auto pub = pub_node->create_publisher<Msg>(topic, 10);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  for (int i = 0; i < 500 && !done(); ++i) {
    pub->publish(msg);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(done());
}

}  // namespace

TEST(TopicTerm, ReportsItsReceiveTimeAndItsMaxAge)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"p.terms.x.topic", "/test_term_max_age"},
      {"p.terms.x.max_age", 0.05}, {"p.terms.x.max_age_on_fault", "damp"}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_term_max_age", opts);
  DataStore state;
  LinearXTerm term;
  term.configure({node, "p.terms.x"}, ModelInfo{}, state);
  EXPECT_DOUBLE_EQ(term.max_age(), 0.05);
  EXPECT_EQ(term.max_age_on_fault(), policy_framework::FaultReaction::DAMP);
  EXPECT_DOUBLE_EQ(term.receive_time(), 0.0);  // nothing yet
  const double before = node->now().seconds();
  Twist msg;
  publish_until(node, "/test_term_max_age", msg, [&]() {return term.ready(state);});
  EXPECT_GE(term.receive_time(), before);
}

TEST(TopicTerm, HasNoMaxAgeByDefault)
{
  auto node = command_node("test_term_no_max_age", "/test_term_no_max_age");
  DataStore state;
  LinearXTerm term;
  term.configure({node, "p.terms.cmd"}, ModelInfo{}, state);
  EXPECT_DOUBLE_EQ(term.max_age(), 0.0);
  EXPECT_EQ(term.max_age_on_fault(), policy_framework::FaultReaction::ERROR);
}

TEST(TopicTerm, RefusesNeutralCommandsForItsMaxAge)
{
  // A stale sensor is not answered by neutral commands: the policy would run on stale data.
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"p.terms.x.max_age", 0.05},
      {"p.terms.x.max_age_on_fault", "neutral_commands"}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_term_neutral", opts);
  DataStore state;
  LinearXTerm term;
  EXPECT_THROW(term.configure({node, "p.terms.x"}, ModelInfo{}, state), std::runtime_error);
}

TEST(CommandTerm, RefusesAMaxAge)
{
  // A command is old by design; its timeout handles an operator who stopped sending.
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"p.terms.cmd.max_age", 0.1}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_command_max_age", opts);
  DataStore state;
  VxCommand term;
  EXPECT_THROW(term.configure({node, "p.terms.cmd"}, ModelInfo{}, state), std::runtime_error);
}

TEST(CommandTerm, AnOrdinaryTopicTermIsNotAUserCommand)
{
  EXPECT_FALSE(LinearXTerm().user_command());
  EXPECT_TRUE(VxCommand().user_command());
}

TEST(CommandTerm, IsReadyAndNeutralWithoutACommandStampedNow)
{
  auto node = command_node("test_command_none", "/test_command_none");
  DataStore state;
  VxCommand term;
  ASSERT_EQ(term.configure({node, "p.terms.cmd"}, ModelInfo{}, state), 1U);
  EXPECT_TRUE(term.ready(state));  // a policy does not wait for a command
  float out = 9.0F;
  const double before = node->now().seconds();
  EXPECT_GE(term.compute(state, TensorMap{}, &out), before);
  EXPECT_FLOAT_EQ(out, 0.0F);
}

TEST(CommandTerm, FollowsTheCommandStampedNowUntilItsTimeout)
{
  auto node = command_node("test_command_timeout", "/test_command_timeout");
  DataStore state;
  VxCommand term;
  term.configure({node, "p.terms.cmd"}, ModelInfo{}, state);
  Twist msg;
  msg.linear.x = 0.5;
  float out = 0.0F;
  publish_until(node, "/test_command_timeout", msg, [&]() {
      term.compute(state, TensorMap{}, &out);
      return out > 0.4F;
    });
  // A Twist has no header: the command in effect is stamped now, not with its receive time.
  const double before = node->now().seconds();
  EXPECT_GE(term.compute(state, TensorMap{}, &out), before);
  EXPECT_FLOAT_EQ(out, 0.5F);
  std::this_thread::sleep_for(std::chrono::milliseconds(400));  // older than the 0.3 s timeout
  term.compute(state, TensorMap{}, &out);
  EXPECT_FLOAT_EQ(out, 0.0F);
}

TEST(CommandTerm, StampsACommandWithItsHeader)
{
  auto node = command_node("test_command_stamped", "/test_command_stamped");
  DataStore state;
  StampedVxCommand term;
  term.configure({node, "p.terms.cmd"}, ModelInfo{}, state);
  TwistStamped msg;
  msg.header.stamp = rclcpp::Time(1234, 500000000, RCL_ROS_TIME);
  msg.twist.linear.x = 0.5;
  float out = 0.0F;
  double stamp = 0.0;
  publish_until(node, "/test_command_stamped", msg, [&]() {
      stamp = term.compute(state, TensorMap{}, &out);
      return out > 0.4F;
    });
  EXPECT_DOUBLE_EQ(stamp, 1234.5);
}

TEST(CommandTerm, IsNeutralWhileTheGateIsClosed)
{
  auto node = command_node("test_command_gate", "/test_command_gate");
  DataStore state;
  const auto safe = state.add("safety/safe");
  VxCommand term;
  term.configure({node, "p.terms.cmd", nullptr, "safety/safe"}, ModelInfo{}, state);
  Twist msg;
  msg.linear.x = 0.5;
  float out = 0.0F;
  state.set(safe, 1.0, 1.0);
  publish_until(node, "/test_command_gate", msg, [&]() {
      term.compute(state, TensorMap{}, &out);
      return out > 0.4F;
    });
  state.set(safe, 0.0, 1.0);
  term.compute(state, TensorMap{}, &out);
  EXPECT_FLOAT_EQ(out, 0.0F);
}

TEST(CommandTerm, RejectsADisabledTimeout)
{
  // Without a timeout a lost operator would keep the last command applied for ever.
  auto node = command_node("test_command_no_timeout", "/test_command_no_timeout", 0.0);
  DataStore state;
  VxCommand term;
  EXPECT_THROW(term.configure({node, "p.terms.cmd"}, ModelInfo{}, state), std::runtime_error);
}

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
  const double stamp = term.compute(state, TensorMap{}, &out);
  EXPECT_FLOAT_EQ(out, 0.25F);
  // The receive time, on the node clock.
  EXPECT_GT(stamp, 0.0);
  EXPECT_LE(stamp, node->now().seconds());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
