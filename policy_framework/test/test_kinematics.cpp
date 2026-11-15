// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <future>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "policy_framework/kinematics.hpp"

using policy_framework::Kinematics;
using StringArray = std::vector<std::string>;

namespace
{

std::string read_urdf()
{
  std::ifstream f(TEST_URDF);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

/// Vector in the order of kin.joint_names() from values by joint name (0 when absent).
Eigen::VectorXd by_name(const Kinematics & kin, const std::map<std::string, double> & values)
{
  Eigen::VectorXd v = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(kin.joint_names().size()));
  for (size_t i = 0; i < kin.joint_names().size(); ++i) {
    const auto it = values.find(kin.joint_names()[i]);
    if (it != values.end()) {v[static_cast<Eigen::Index>(i)] = it->second;}
  }
  return v;
}

}  // namespace

TEST(Kinematics, ExposesEveryMovableJointOfTheWholeBody)
{
  Kinematics kin(read_urdf());
  auto names = kin.joint_names();
  std::sort(names.begin(), names.end());
  EXPECT_EQ(names, (StringArray{"j1", "j2", "j3"}));
}

TEST(Kinematics, IsNotReadyBeforeTheFirstUpdate)
{
  Kinematics kin(read_urdf());
  EXPECT_FALSE(kin.ready());
}

TEST(Kinematics, ComputesTheTransformOfAnyLinkOfTheBody)
{
  Kinematics kin(read_urdf());
  const auto imu = kin.frame("imu_link");
  const auto link1 = kin.frame("link1");
  const auto side = kin.frame("side_link");
  const double q1 = M_PI / 2;
  const double q2 = 0.3;
  const double q3 = M_PI / 2;
  const auto zero = by_name(kin, {});
  ASSERT_TRUE(kin.update(by_name(kin, {{"j1", q1}, {"j2", q2}, {"j3", q3}}), zero, 4.0));
  ASSERT_TRUE(kin.ready());
  EXPECT_DOUBLE_EQ(kin.stamp(), 4.0);
  // p = Rz(q1) * ((1, 0, 0) + Ry(q2) * (0, 0, 1))
  const Eigen::Vector3d p = kin.transform(imu).translation();
  EXPECT_NEAR(p.x(), 0.0, 1e-9);
  EXPECT_NEAR(p.y(), 1.0 + std::sin(q2), 1e-9);
  EXPECT_NEAR(p.z(), std::cos(q2), 1e-9);
  EXPECT_TRUE(
    kin.transform(link1).linear().isApprox(
      Eigen::AngleAxisd(q1, Eigen::Vector3d::UnitZ()).toRotationMatrix()));
  // Continuous joint on another branch.
  EXPECT_TRUE(kin.transform(side).translation().isApprox(Eigen::Vector3d(0, 2, 0)));
  EXPECT_TRUE(
    kin.transform(side).linear().isApprox(
      Eigen::AngleAxisd(q3, Eigen::Vector3d::UnitX()).toRotationMatrix()));
  // Relative transform: side_link in the frame of link1.
  EXPECT_TRUE(
    kin.transform(side, link1).isApprox(kin.transform(link1).inverse() * kin.transform(side)));
}

TEST(Kinematics, ComputesFrameVelocitiesFromTheJointVelocities)
{
  Kinematics kin(read_urdf());
  const auto imu = kin.frame("imu_link");
  ASSERT_TRUE(kin.update(by_name(kin, {}), by_name(kin, {{"j1", 2.0}}), 1.0));
  // imu_link at (1, 0, 1), turning at 2 rad/s about z: v = w x p = (0, 2, 0).
  const auto v = kin.velocity(imu, Kinematics::Reference::ROOT_ALIGNED);
  EXPECT_TRUE(v.head<3>().isApprox(Eigen::Vector3d(0, 2, 0)));
  EXPECT_TRUE(v.tail<3>().isApprox(Eigen::Vector3d(0, 0, 2)));
  // In imu_link's own axes: the same velocity, rotated.
  const auto local = kin.velocity(imu, Kinematics::Reference::LOCAL);
  EXPECT_TRUE(local.head<3>().isApprox(kin.transform(imu).linear().transpose() * v.head<3>()));
}

TEST(Kinematics, JointsNotSelectedAreLockedAtZero)
{
  Kinematics kin(read_urdf(), {"j2", "j1"});
  auto names = kin.joint_names();
  std::sort(names.begin(), names.end());
  EXPECT_EQ(names, (StringArray{"j1", "j2"}));
  const auto side = kin.frame("side_link");
  ASSERT_TRUE(kin.update(by_name(kin, {{"j1", 0.5}}), by_name(kin, {}), 1.0));
  EXPECT_TRUE(kin.transform(side).linear().isIdentity(1e-12));
}

TEST(Kinematics, WorldOrientationComposesAnImuOrientationWithTheBody)
{
  Kinematics kin(read_urdf());
  const auto base = kin.frame("base_link");
  const auto link2 = kin.frame("link2");
  const auto imu = kin.frame("imu_link");
  ASSERT_TRUE(kin.update(by_name(kin, {{"j1", 0.4}, {"j2", -0.2}}), by_name(kin, {}), 1.0));
  // IMU yawed 90 deg in the world, as is imu_link w.r.t. link2: link2 then has yaw 0, and the
  // base is rotated back by the joints: Rz(-0.4) after undoing Ry(-0.2).
  const Eigen::Quaterniond q_imu(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(kin.world_orientation(link2, imu, q_imu).isApprox(Eigen::Quaterniond::Identity()));
  const Eigen::Quaterniond expected_base =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(kin.world_orientation(base, imu, q_imu).isApprox(expected_base));
}

TEST(Kinematics, WorldAngularVelocityRemovesTheJointMotionFromAGyroscope)
{
  Kinematics kin(read_urdf());
  const auto base = kin.frame("base_link");
  const auto link2 = kin.frame("link2");
  const auto imu = kin.frame("imu_link");
  ASSERT_TRUE(
    kin.update(
      by_name(kin, {{"j1", 0.4}, {"j2", -0.2}}), by_name(kin, {{"j1", 2.0}, {"j2", -1.0}}), 1.0));

  // The base turns at `base_rate` (in its own axes) while the joints turn the IMU further:
  // j1 about the base z, j2 about the y axis of link1, which j1 has turned by 0.4 rad.
  const Eigen::Vector3d base_rate(0.1, -0.3, 0.5);
  const Eigen::Vector3d joints_rate = 2.0 * Eigen::Vector3d::UnitZ() +
    -1.0 * (Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitY());
  const Eigen::Matrix3d base_from_imu = kin.transform(imu).linear();
  const Eigen::Vector3d gyro = base_from_imu.transpose() * (base_rate + joints_rate);

  // The root link does not move in the model, whatever the joints do.
  EXPECT_TRUE(kin.velocity(base, Kinematics::Reference::LOCAL).isZero());
  EXPECT_TRUE(kin.world_angular_velocity(base, imu, gyro).isApprox(base_rate, 1e-9));
  // link2 carries the IMU, yawed by 90 degrees: the same rate in other axes.
  const Eigen::Vector3d link2_rate = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) * gyro;
  EXPECT_TRUE(kin.world_angular_velocity(link2, imu, gyro).isApprox(link2_rate, 1e-9));
}

TEST(Kinematics, UpdateIsSkippedWithoutBlockingWhileAReaderHoldsTheLock)
{
  Kinematics kin(read_urdf());
  const auto zero = by_name(kin, {});
  std::promise<void> locked;
  std::promise<void> release;
  std::thread reader([&] {
      kin.with_lock(
        [&](const Kinematics &) {
          locked.set_value();
          release.get_future().wait();
        });
    });
  locked.get_future().wait();
  EXPECT_FALSE(kin.update(zero, zero, 1.0));
  release.set_value();
  reader.join();
  EXPECT_FALSE(kin.ready());  // the skipped update changed nothing
  EXPECT_TRUE(kin.update(zero, zero, 1.0));
  EXPECT_TRUE(kin.ready());
}

TEST(Kinematics, RejectsInvalidInputs)
{
  EXPECT_THROW(Kinematics(""), std::runtime_error);
  EXPECT_THROW(Kinematics(read_urdf(), {"j1", "no_such_joint"}), std::runtime_error);
  Kinematics kin(read_urdf());
  EXPECT_THROW(kin.frame("no_such_link"), std::runtime_error);
  EXPECT_THROW(kin.frame("j1"), std::runtime_error);  // a joint, not a link
  const Eigen::VectorXd wrong = Eigen::VectorXd::Zero(1);
  EXPECT_THROW(kin.update(wrong, wrong, 1.0), std::invalid_argument);
}
