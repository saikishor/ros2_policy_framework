// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "policy_framework/safety_checks.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using policy_framework::DataStore;
using policy_framework::FaultReaction;
using policy_framework::StateBoundsCheck;
using StringArray = std::vector<std::string>;

namespace
{

class StateBoundsCheckTest : public ::testing::Test
{
protected:
  void configure(std::vector<rclcpp::Parameter> extra)
  {
    std::vector<rclcpp::Parameter> params = {
      {"motors.prefixes", StringArray{"j1", "j2"}},
      {"motors.interfaces", StringArray{"temperature", "error_code"}},
      {"motors.temperature.max", 80.0},
      {"motors.error_code.min", 0.0},
      {"motors.error_code.max", 0.0},
    };
    params.insert(params.end(), extra.begin(), extra.end());
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(params);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("bounds", opts);
    check.configure({node, "motors"}, commands, state);
  }

  void set(const std::string & name, double v) {state.set(state.index(name), v, 0.0);}
  bool ok()
  {
    verdict = check.check(state, 0.0, reason);
    return verdict.safe;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  StateBoundsCheck check;
  DataStore commands;
  DataStore state;
  std::string reason;
  policy_framework::SafetyCheck::Verdict verdict;
};

TEST_F(StateBoundsCheckTest, RegistersAnEntryPerPrefixAndInterface)
{
  configure({});
  for (const auto * n : {"j1/temperature", "j1/error_code", "j2/temperature", "j2/error_code"}) {
    EXPECT_TRUE(state.find(n).has_value()) << n;
  }
}

TEST_F(StateBoundsCheckTest, FaultsOnAValueOutsideItsBounds)
{
  configure({});
  EXPECT_TRUE(ok());  // nothing received yet: nothing to check
  set("j1/temperature", 75.0);
  set("j2/error_code", 0.0);
  EXPECT_TRUE(ok());
  set("j2/temperature", 81.0);
  EXPECT_FALSE(ok());
  EXPECT_NE(reason.find("j2/temperature"), std::string::npos) << reason;
  set("j2/temperature", 70.0);
  set("j1/error_code", 3.0);  // any error code but 0
  EXPECT_FALSE(ok());
  EXPECT_NE(reason.find("j1/error_code"), std::string::npos) << reason;
}

TEST_F(StateBoundsCheckTest, FaultsOnANonFiniteValue)
{
  configure({});
  set("j1/temperature", std::nan(""));
  EXPECT_FALSE(ok());
}

TEST_F(StateBoundsCheckTest, FaultsOnlyAfterTriggerCyclesViolationsInARow)
{
  configure({{"motors.trigger_cycles", 3}});
  set("j1/temperature", 90.0);
  EXPECT_TRUE(ok());
  EXPECT_TRUE(ok());
  set("j1/temperature", 70.0);  // a good value starts the count again
  EXPECT_TRUE(ok());
  set("j1/temperature", 90.0);
  EXPECT_TRUE(ok());
  EXPECT_TRUE(ok());
  EXPECT_FALSE(ok());

  check.reset();
  EXPECT_TRUE(ok());
}

TEST_F(StateBoundsCheckTest, PassesAgainOnlyInsideTheBoundsNarrowedByTheHysteresis)
{
  configure({{"motors.temperature.hysteresis", 10.0}});
  set("j1/temperature", 81.0);
  EXPECT_FALSE(ok());
  set("j1/temperature", 75.0);  // back below max, but not below max - hysteresis
  EXPECT_FALSE(ok());
  EXPECT_NE(reason.find("70.0"), std::string::npos) << reason;  // the bound to get back within
  set("j1/temperature", 69.0);
  EXPECT_TRUE(ok());
  set("j1/temperature", 75.0);  // not failed any more: the plain bounds again
  EXPECT_TRUE(ok());

  set("j1/temperature", 81.0);
  EXPECT_FALSE(ok());
  check.reset();  // forgets that it failed
  set("j1/temperature", 75.0);
  EXPECT_TRUE(ok());
}

TEST_F(StateBoundsCheckTest, RejectsAHysteresisThatLeavesNothingToPass)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    {"motors.prefixes", StringArray{"j1"}},
    {"motors.interfaces", StringArray{"error_code"}},
    {"motors.error_code.min", 0.0},
    {"motors.error_code.max", 1.0},
    {"motors.error_code.hysteresis", 0.6}});
  node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("empty_band", opts);
  EXPECT_THROW(check.configure({node, "motors"}, commands, state), std::runtime_error);
}

TEST_F(StateBoundsCheckTest, AsksForNoOtherReactionWithoutEscalation)
{
  configure({});
  EXPECT_TRUE(check.possible_reactions().empty());
  set("j1/temperature", 81.0);
  set("j2/temperature", 81.0);
  EXPECT_FALSE(ok());
  EXPECT_FALSE(verdict.reaction.has_value());  // the configured on_fault
}

TEST(FaultReaction, IsParsedFromItsConfigurationName)
{
  using policy_framework::fault_reaction_from_string;
  EXPECT_EQ(fault_reaction_from_string("neutral_commands"), FaultReaction::NEUTRAL_COMMANDS);
  EXPECT_EQ(fault_reaction_from_string("hold"), FaultReaction::HOLD);
  EXPECT_EQ(fault_reaction_from_string("damp"), FaultReaction::DAMP);
  EXPECT_EQ(fault_reaction_from_string("safe"), FaultReaction::SAFE);
  EXPECT_EQ(fault_reaction_from_string("error"), FaultReaction::ERROR);
  EXPECT_FALSE(fault_reaction_from_string("ERROR").has_value());
  EXPECT_FALSE(fault_reaction_from_string("").has_value());
}

TEST_F(StateBoundsCheckTest, RejectsAnInterfaceWithoutBounds)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    {"motors.prefixes", StringArray{"j1"}},
    {"motors.interfaces", StringArray{"temperature"}}});
  node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("no_bounds", opts);
  EXPECT_THROW(check.configure({node, "motors"}, commands, state), std::runtime_error);
}

rclcpp_lifecycle::LifecycleNode::SharedPtr make_node(
  const std::string & name, const std::vector<rclcpp::Parameter> & params)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, opts);
}

class JumpCheckTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto node = make_node(
      "jump", {
        {"encoders.prefixes", StringArray{"j1", "j2"}},
        {"encoders.interfaces", StringArray{"position"}},
        {"encoders.position.max_delta", 0.1}});
    check.configure({node, "encoders"}, commands, state);
  }

  void set(const std::string & name, double v) {state.set(state.index(name), v, 0.0);}
  bool ok()
  {
    verdict = check.check(state, 0.0, reason);
    return verdict.safe;
  }

  policy_framework::JumpCheck check;
  DataStore commands;
  DataStore state;
  std::string reason;
  policy_framework::SafetyCheck::Verdict verdict;
};

TEST_F(JumpCheckTest, FaultsOnAChangeAboveMaxDeltaBetweenTwoCycles)
{
  EXPECT_TRUE(ok());  // nothing received yet
  set("j1/position", 1.0);  // a first value is never a jump
  EXPECT_TRUE(ok());
  set("j1/position", 1.05);
  set("j2/position", -3.0);
  EXPECT_TRUE(ok());
  set("j2/position", -2.8);
  EXPECT_FALSE(ok());
  EXPECT_NE(reason.find("j2/position"), std::string::npos) << reason;
}

TEST_F(JumpCheckTest, FaultsOnANonFiniteValue)
{
  set("j1/position", std::nan(""));  // even as a first value
  EXPECT_FALSE(ok());
  check.reset();
  set("j1/position", 1.0);
  EXPECT_TRUE(ok());
  set("j1/position", std::nan(""));
  EXPECT_FALSE(ok());
}

TEST_F(JumpCheckTest, ResetForgetsThePreviousValues)
{
  set("j1/position", 1.0);
  EXPECT_TRUE(ok());
  check.reset();
  set("j1/position", 2.0);  // compared with nothing after the reset
  EXPECT_TRUE(ok());
}

TEST(JumpCheckConfigTest, RejectsAnInterfaceWithoutMaxDelta)
{
  const auto node = make_node(
    "no_delta", {
      {"encoders.prefixes", StringArray{"j1"}},
      {"encoders.interfaces", StringArray{"position"}}});
  policy_framework::JumpCheck check;
  DataStore commands, state;
  EXPECT_THROW(check.configure({node, "encoders"}, commands, state), std::runtime_error);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
