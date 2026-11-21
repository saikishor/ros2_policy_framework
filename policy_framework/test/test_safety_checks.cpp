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
  bool ok() {return check.check(state, 0.0, reason);}

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  StateBoundsCheck check;
  DataStore commands;
  DataStore state;
  std::string reason;
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

TEST_F(StateBoundsCheckTest, RejectsAnInterfaceWithoutBounds)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    {"motors.prefixes", StringArray{"j1"}},
    {"motors.interfaces", StringArray{"temperature"}}});
  node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("no_bounds", opts);
  EXPECT_THROW(check.configure({node, "motors"}, commands, state), std::runtime_error);
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
