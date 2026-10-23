// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "policy_framework/kinematics.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::DataStore;
using policy_framework::Kinematics;

namespace
{

std::string read_urdf()
{
  std::ifstream f(TEST_URDF);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

rclcpp_lifecycle::LifecycleNode::SharedPtr make_node(std::vector<rclcpp::Parameter> params)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_kinematics", opts);
}

std::vector<rclcpp::Parameter> kdl_params(const std::string & tip = "imu_link")
{
  return {{"kinematics.plugin_name", "kinematics_interface_kdl/KinematicsInterfaceKDL"},
    {"kinematics.base", "base_link"}, {"kinematics.tip", tip}};
}

void set(DataStore & s, const std::string & name, double v, double stamp = 1.0)
{
  s.set(s.index(name), v, stamp);
}

}  // namespace

TEST(Kinematics, IsNotCreatedWithoutPlugin)
{
  DataStore state;
  EXPECT_EQ(Kinematics::create(make_node({}), read_urdf(), state), nullptr);
  EXPECT_EQ(state.size(), 0u);
}

TEST(Kinematics, RegistersTheChainJointPositionsInChainOrder)
{
  DataStore state;
  const auto kin = Kinematics::create(make_node(kdl_params()), read_urdf(), state);
  ASSERT_NE(kin, nullptr);
  EXPECT_EQ(kin->joint_names(), (std::vector<std::string>{"j1", "j2"}));
  EXPECT_TRUE(state.find("j1/position").has_value());
  EXPECT_TRUE(state.find("j2/position").has_value());
}

TEST(Kinematics, ComputesTheTransformOfRegisteredFramesFromTheState)
{
  DataStore state;
  const auto kin = Kinematics::create(make_node(kdl_params()), read_urdf(), state);
  const auto imu = kin->frame("imu_link");
  const auto link1 = kin->frame("link1");
  const double q1 = M_PI / 2;
  const double q2 = 0.3;
  set(state, "j1/position", q1);
  set(state, "j2/position", q2);
  kin->update(state);
  ASSERT_TRUE(kin->ready());
  // p = Rz(q1) * ((1, 0, 0) + Ry(q2) * (0, 0, 1))
  const Eigen::Vector3d p = kin->transform(imu).translation();
  EXPECT_NEAR(p.x(), 0.0, 1e-9);
  EXPECT_NEAR(p.y(), 1.0 + std::sin(q2), 1e-9);
  EXPECT_NEAR(p.z(), std::cos(q2), 1e-9);
  const Eigen::Matrix3d r1 = kin->transform(link1).rotation();
  EXPECT_TRUE(r1.isApprox(Eigen::AngleAxisd(q1, Eigen::Vector3d::UnitZ()).toRotationMatrix()));
}

TEST(Kinematics, RejectsLinksOutsideTheChain)
{
  DataStore state;
  const auto kin = Kinematics::create(make_node(kdl_params()), read_urdf(), state);
  EXPECT_THROW(kin->frame("side_link"), std::runtime_error);
  EXPECT_THROW(kin->frame("no_such_link"), std::runtime_error);
}

TEST(Kinematics, IsReadyOnceEveryJointWasReceivedAndStampedWithTheOldest)
{
  DataStore state;
  const auto kin = Kinematics::create(make_node(kdl_params()), read_urdf(), state);
  kin->frame("imu_link");
  kin->update(state);
  EXPECT_FALSE(kin->ready());
  set(state, "j1/position", 0.0, 3.0);
  kin->update(state);
  EXPECT_FALSE(kin->ready());
  set(state, "j2/position", 0.0, 2.0);
  kin->update(state);
  EXPECT_TRUE(kin->ready());
  EXPECT_DOUBLE_EQ(kin->stamp(), 2.0);
}

TEST(Kinematics, WorldOrientationComposesTheImuOrientationWithTheChain)
{
  auto params = kdl_params();
  params.emplace_back("kinematics.imu_link", "imu_link");
  params.emplace_back("kinematics.imu_sensor", "imu");
  DataStore state;
  const auto kin = Kinematics::create(make_node(params), read_urdf(), state);
  const auto base = kin->frame("base_link");
  const auto link2 = kin->frame("link2");
  set(state, "j1/position", 0.4);
  set(state, "j2/position", -0.2);
  // IMU yawed 90 deg in the world, as is imu_link w.r.t. link2: link2 then has yaw 0, and the
  // base is rotated back by the joints: Rz(-0.4) after undoing Ry(-0.2).
  const Eigen::Quaterniond q_imu(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  set(state, "imu/orientation.x", q_imu.x(), 5.0);
  set(state, "imu/orientation.y", q_imu.y(), 5.0);
  set(state, "imu/orientation.z", q_imu.z(), 5.0);
  set(state, "imu/orientation.w", q_imu.w(), 5.0);
  kin->update(state);
  ASSERT_TRUE(kin->ready());
  EXPECT_TRUE(kin->world_orientation(link2).isApprox(Eigen::Quaterniond::Identity()));
  const Eigen::Quaterniond expected_base =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(kin->world_orientation(base).isApprox(expected_base))
    << kin->world_orientation(base).coeffs().transpose();
}

TEST(Kinematics, RejectsAnInvalidConfiguration)
{
  DataStore state;
  // No robot description.
  EXPECT_THROW(Kinematics::create(make_node(kdl_params()), "", state), std::runtime_error);
  // Missing tip.
  EXPECT_THROW(Kinematics::create(make_node(kdl_params("")), read_urdf(), state), std::runtime_error);
  // The IMU link must be on the chain.
  auto params = kdl_params();
  params.emplace_back("kinematics.imu_link", "side_link");
  params.emplace_back("kinematics.imu_sensor", "imu");
  EXPECT_THROW(Kinematics::create(make_node(params), read_urdf(), state), std::runtime_error);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
