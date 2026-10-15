// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "policy_framework/interface_executors.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::DataStore;
using policy_framework::PdEffortExecutor;
using policy_framework::PluginContext;
using StringArray = std::vector<std::string>;
using Doubles = std::vector<double>;

namespace
{

PluginContext make_context(const std::vector<rclcpp::Parameter> & params)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  return {std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_interface_executors", opts),
    "pd"};
}

void set(DataStore & s, const std::string & name, double v) {s.set(s.index(name), v, 1.0);}

class PdEffortExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    for (const auto * n : {"a/position", "a/velocity", "b/position", "b/velocity"}) {
      commands.add(n);
    }
  }

  void configure(const std::vector<rclcpp::Parameter> & params)
  {
    pd.configure(make_context(params), commands, state);
    values.assign(pd.command_interfaces().size(), 0.0);
  }

  /// Robot at q = (1, 2), dq = (0.5, -1); references q = (1.5, 1), dq = (0, 0).
  void set_robot_and_references()
  {
    set(state, "a/position", 1.0);
    set(state, "a/velocity", 0.5);
    set(state, "b/position", 2.0);
    set(state, "b/velocity", -1.0);
    set(commands, "a/position", 1.5);
    set(commands, "a/velocity", 0.0);
    set(commands, "b/position", 1.0);
    set(commands, "b/velocity", 0.0);
  }

  DataStore commands;
  DataStore state;
  PdEffortExecutor pd;
  Doubles values;
};

}  // namespace

TEST_F(PdEffortExecutorTest, ClaimsEffortOfEveryPositionCommandAndReadsJointStates)
{
  configure({{"pd.stiffness", Doubles{10.0, 20.0}}, {"pd.damping", Doubles{1.0, 2.0}}});
  EXPECT_EQ(pd.command_interfaces(), (StringArray{"a/effort", "b/effort"}));
  for (const auto * n : {"a/position", "a/velocity", "b/position", "b/velocity"}) {
    EXPECT_TRUE(state.find(n).has_value()) << n;
  }
}

TEST_F(PdEffortExecutorTest, ComputesPdEffortWithParameterGains)
{
  configure({{"pd.stiffness", Doubles{10.0, 20.0}}, {"pd.damping", Doubles{1.0, 2.0}}});
  set_robot_and_references();
  pd.write(commands, state, values);
  EXPECT_DOUBLE_EQ(values[0], 10.0 * (1.5 - 1.0) + 1.0 * (0.0 - 0.5));
  EXPECT_DOUBLE_EQ(values[1], 20.0 * (1.0 - 2.0) + 2.0 * (0.0 + 1.0));
}

TEST_F(PdEffortExecutorTest, GainsWrittenByThePolicyOverrideTheParameters)
{
  for (const auto * n : {"a/stiffness", "a/damping", "b/stiffness", "b/damping"}) {
    commands.add(n);
  }
  configure({});  // no parameter gains needed when the policy writes them
  set_robot_and_references();
  set(commands, "a/stiffness", 4.0);
  set(commands, "a/damping", 0.5);
  set(commands, "b/stiffness", 8.0);
  set(commands, "b/damping", 1.0);
  pd.write(commands, state, values);
  EXPECT_DOUBLE_EQ(values[0], 4.0 * 0.5 + 0.5 * -0.5);
  EXPECT_DOUBLE_EQ(values[1], 8.0 * -1.0 + 1.0 * 1.0);
}

TEST_F(PdEffortExecutorTest, ClampsToTheEffortLimits)
{
  configure({{"pd.stiffness", Doubles{100.0, 100.0}}, {"pd.damping", Doubles{0.0, 0.0}},
      {"pd.effort_limits", Doubles{10.0, 30.0}}});
  set_robot_and_references();
  pd.write(commands, state, values);
  EXPECT_DOUBLE_EQ(values[0], 10.0);   // 50 clamped
  EXPECT_DOUBLE_EQ(values[1], -30.0);  // -100 clamped
}

TEST_F(PdEffortExecutorTest, WritesNothingForAJointWithMissingOrInvalidData)
{
  configure({{"pd.stiffness", Doubles{10.0, 20.0}}, {"pd.damping", Doubles{1.0, 2.0}}});
  pd.write(commands, state, values);  // nothing received yet
  EXPECT_TRUE(std::isnan(values[0]));
  EXPECT_TRUE(std::isnan(values[1]));

  set_robot_and_references();
  set(state, "b/position", std::numeric_limits<double>::quiet_NaN());
  pd.write(commands, state, values);
  EXPECT_FALSE(std::isnan(values[0]));
  EXPECT_TRUE(std::isnan(values[1]));
}

TEST_F(PdEffortExecutorTest, SafeValuesAreZeroEffort)
{
  configure({{"pd.stiffness", Doubles{10.0, 20.0}}, {"pd.damping", Doubles{1.0, 2.0}}});
  values.assign(2, 7.0);
  pd.safe_values(values);
  EXPECT_EQ(values, (Doubles{0.0, 0.0}));
}

TEST_F(PdEffortExecutorTest, RejectsMissingGainsAndSizeMismatch)
{
  EXPECT_THROW(configure({}), std::runtime_error);  // no gains from the policy nor parameters
  PdEffortExecutor other;
  EXPECT_THROW(
    other.configure(
      make_context({{"pd.stiffness", Doubles{1.0}}, {"pd.damping", Doubles{1.0, 1.0}}}),
      commands, state), std::runtime_error);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
