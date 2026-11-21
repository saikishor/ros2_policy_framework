// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "policy_framework/safety_monitor.hpp"

using policy_framework::DataStore;
using policy_framework::SafetyMonitor;

namespace
{

class SafetyMonitorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    commands.add("j1/position");
    commands.add("j2/position");
    commands.add("j1/stiffness");
  }

  void configure() {monitor.configure(config, commands, state);}

  void set(const std::string & name, double v) {state.set(state.index(name), v, 0.0);}

  /// IMU orientation turned by @p angle about the x axis.
  void tilt_imu(double angle)
  {
    set("imu/orientation.x", std::sin(angle / 2));
    set("imu/orientation.y", 0.0);
    set("imu/orientation.z", 0.0);
    set("imu/orientation.w", std::cos(angle / 2));
  }

  SafetyMonitor::Config config;
  SafetyMonitor monitor;
  DataStore commands;
  DataStore state;
};

}  // namespace

TEST_F(SafetyMonitorTest, EveryCheckIsOffByDefault)
{
  configure();
  EXPECT_EQ(state.size(), 0u);  // reads nothing
  EXPECT_FALSE(monitor.check(state, 1.0, 1000.0));
}

TEST_F(SafetyMonitorTest, DetectsATiltAboveTheLimit)
{
  config.max_tilt = 0.5;
  config.imu = "imu";
  configure();
  EXPECT_TRUE(state.find("imu/orientation.w").has_value());
  EXPECT_FALSE(monitor.check(state, 0.0, 0.0));  // no orientation yet: nothing to check

  tilt_imu(0.4);
  EXPECT_FALSE(monitor.check(state, 0.0, 0.0));
  tilt_imu(-0.6);
  const auto v = monitor.check(state, 0.0, 0.0);
  ASSERT_TRUE(v);
  EXPECT_EQ(v->check, SafetyMonitor::Check::TILT);
  EXPECT_NEAR(v->value, 0.6, 1e-9);
  EXPECT_NE(v->describe().find("tilt"), std::string::npos);
}

TEST_F(SafetyMonitorTest, TiltNeedsAnImu)
{
  config.max_tilt = 0.5;
  EXPECT_THROW(configure(), std::invalid_argument);
}

TEST_F(SafetyMonitorTest, DetectsAJointVelocityAboveTheLimitOnCommandedJoints)
{
  config.max_joint_velocity = 10.0;
  configure();
  EXPECT_TRUE(state.find("j1/velocity").has_value());
  EXPECT_TRUE(state.find("j2/velocity").has_value());
  EXPECT_EQ(state.size(), 2u);

  set("j1/velocity", 9.0);
  EXPECT_FALSE(monitor.check(state, 0.0, 0.0));
  set("j2/velocity", -11.0);
  const auto v = monitor.check(state, 0.0, 0.0);
  ASSERT_TRUE(v);
  EXPECT_EQ(v->check, SafetyMonitor::Check::JOINT_VELOCITY);
  EXPECT_EQ(v->joint, "j2");
  EXPECT_DOUBLE_EQ(v->value, -11.0);
}

TEST_F(SafetyMonitorTest, DetectsAJointPositionOutsideItsLimitsMinusTheMargin)
{
  config.joint_limits = {{"j1", {-1.0, 1.0}}, {"j3", {0.0, 0.1}}};  // j3 is not commanded
  config.position_margin = 0.1;
  configure();
  EXPECT_TRUE(state.find("j1/position").has_value());
  EXPECT_FALSE(state.find("j2/position").has_value());  // no limits
  EXPECT_FALSE(state.find("j3/position").has_value());

  set("j1/position", 0.85);
  EXPECT_FALSE(monitor.check(state, 0.0, 0.0));
  set("j1/position", -0.95);
  const auto v = monitor.check(state, 0.0, 0.0);
  ASSERT_TRUE(v);
  EXPECT_EQ(v->check, SafetyMonitor::Check::JOINT_POSITION);
  EXPECT_EQ(v->joint, "j1");
}

TEST_F(SafetyMonitorTest, DetectsAnObservationOlderThanTheLimit)
{
  config.max_input_age = 0.2;
  configure();
  EXPECT_FALSE(monitor.check(state, 0.0, 10.0));  // no step yet: unknown age
  EXPECT_FALSE(monitor.check(state, 9.9, 10.0));
  const auto v = monitor.check(state, 9.7, 10.0);
  ASSERT_TRUE(v);
  EXPECT_EQ(v->check, SafetyMonitor::Check::INPUT_AGE);
  EXPECT_NEAR(v->value, 0.3, 1e-9);
}

TEST(SafetyMonitor, ReadsTheJointLimitsOfARobotDescription)
{
  std::ifstream f(TEST_URDF);
  std::stringstream urdf;
  urdf << f.rdbuf();
  const auto limits = SafetyMonitor::joint_limits(urdf.str());
  ASSERT_EQ(limits.count("j1"), 1u);
  EXPECT_DOUBLE_EQ(limits.at("j1").first, -3.2);
  EXPECT_DOUBLE_EQ(limits.at("j2").second, 3.2);
  EXPECT_EQ(limits.count("j3"), 0u);  // continuous: no limits
  EXPECT_THROW(SafetyMonitor::joint_limits("<robot"), std::exception);
}
