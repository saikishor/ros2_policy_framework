// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "policy_framework/kinematics.hpp"
#include "policy_framework_mjlab/motion_goal_term.hpp"
#include "rclcpp/rclcpp.hpp"

using pal_motion_msgs::msg::MotionReference;
using policy_framework::DataStore;
using policy_framework::ModelInfo;
using policy_framework::TensorMap;
using policy_framework_mjlab::MotionGoalTerm;
using Floats = std::vector<float>;

namespace
{

/// One frame over joints {b, a}, with the root turned by `root_yaw` about z.
MotionReference message(double root_yaw)
{
  MotionReference reference;
  reference.joint_names = {"b", "a"};
  reference.frames.resize(1);
  reference.frames.front().positions = {2.0, 1.0};
  reference.frames.front().velocities = {20.0, 10.0};
  reference.root_orientation.resize(1);
  reference.root_orientation.front().w = std::cos(root_yaw / 2);
  reference.root_orientation.front().z = std::sin(root_yaw / 2);
  return reference;
}

// The test robot (policy_framework's test URDF) with its joints at zero: imu_link is yawed by
// 90 degrees w.r.t. base_link, the base of the goal.
class MotionGoalTermTest : public ::testing::Test
{
protected:
  void configure(int frames = 1, const std::string & safety_entry = "")
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({{"p.terms.motion_goal.topic", "/test_motion_reference"},
        {"p.terms.motion_goal.timeout", 0.4}});
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motion_goal", opts);
    pub_node = std::make_shared<rclcpp::Node>("test_motion_goal_pub");
    pub = pub_node->create_publisher<MotionReference>("/test_motion_reference", 10);
    exec.add_node(node->get_node_base_interface());
    std::ifstream f(TEST_URDF);
    std::stringstream urdf;
    urdf << f.rdbuf();
    auto kinematics = std::make_shared<policy_framework::Kinematics>(urdf.str());
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(3);
    ASSERT_TRUE(kinematics->update(zero, zero, 1.0));
    ModelInfo model;
    model.metadata = {
      {"joint_names", "a,b"}, {"default_joint_pos", "0.5,-0.5"}, {"goal_joint_names", "a,b"},
      {"goal_num_future_frames", std::to_string(frames)}, {"goal_root_body", "base_link"}};
    const auto dim = term.configure(
      {node, "p.terms.motion_goal", kinematics, safety_entry}, model, state);
    ASSERT_EQ(dim, static_cast<size_t>(frames) * (2 * 2 + 6));
    out.assign(dim, 9.0F);
    set_robot_yaw(0.0);
    for (const auto * n : {"imu/angular_velocity.x", "imu/angular_velocity.y",
        "imu/angular_velocity.z"})
    {
      state.set(state.index(n), 0.0, 1.0);
    }
  }

  void set_robot_yaw(double yaw)
  {
    const double imu_yaw = yaw + M_PI / 2;
    state.set(state.index("imu/orientation.w"), std::cos(imu_yaw / 2), 1.0);
    state.set(state.index("imu/orientation.x"), 0.0, 1.0);
    state.set(state.index("imu/orientation.y"), 0.0, 1.0);
    state.set(state.index("imu/orientation.z"), std::sin(imu_yaw / 2), 1.0);
  }

  /// Publish until the term outputs the reference (its first joint position is 1).
  void stream(const MotionReference & msg)
  {
    for (int i = 0; i < 500 && std::abs(out[0] - 1.0F) > 1e-6F; ++i) {
      pub->publish(msg);
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      term.compute(state, TensorMap{}, out.data());
    }
    ASSERT_FLOAT_EQ(out[0], 1.0F);
  }

  void expect_out(const Floats & expected)
  {
    ASSERT_EQ(out.size(), expected.size());
    for (size_t i = 0; i < out.size(); ++i) {EXPECT_NEAR(out[i], expected[i], 1e-6) << "at " << i;}
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<MotionReference>::SharedPtr pub;
  rclcpp::executors::SingleThreadedExecutor exec;
  DataStore state;
  MotionGoalTerm term{"imu", "imu_link"};
  Floats out;
};

const Floats kHold = {0.5F, -0.5F, 0, 0, /**/ 1, 0, 0, 0, 1, 0};

}  // namespace

TEST_F(MotionGoalTermTest, WithoutReferenceHoldsTheDefaultPoseAtTheRobotHeading)
{
  configure();
  EXPECT_TRUE(term.ready(state));  // a policy must not wait for a reference to start
  set_robot_yaw(0.7);
  term.compute(state, TensorMap{}, out.data());
  expect_out(kHold);
}

TEST_F(MotionGoalTermTest, AStreamStartsAtTheRobotHeadingAndKeepsThatAlignment)
{
  configure();
  const double robot_yaw = 0.3;
  set_robot_yaw(robot_yaw);
  stream(message(1.0));
  // Reordered to the goal joints {a, b}: positions, then velocities; the root is unrotated.
  expect_out({1, 2, 10, 20, /**/ 1, 0, 0, 0, 1, 0});

  // The robot turns while the reference does not: the goal asks to turn back.
  const double turn = 0.2;
  set_robot_yaw(robot_yaw + turn);
  term.compute(state, TensorMap{}, out.data());
  const auto c = static_cast<float>(std::cos(turn));
  const auto s = static_cast<float>(std::sin(turn));
  expect_out({1, 2, 10, 20, /**/ c, -s, 0, s, c, 0});

  // After reset() (the policy was activated again) the stream is aligned anew.
  term.reset();
  term.compute(state, TensorMap{}, out.data());
  expect_out({1, 2, 10, 20, /**/ 1, 0, 0, 0, 1, 0});
}

TEST_F(MotionGoalTermTest, RepeatsTheLastFrameUpToThePolicyLookahead)
{
  configure(2);
  stream(message(0.0));
  expect_out({1, 2, 10, 20, 1, 0, 0, 0, 1, 0, /**/ 1, 2, 10, 20, 1, 0, 0, 0, 1, 0});
}

TEST_F(MotionGoalTermTest, HoldsAgainWhenTheReferenceIsOldOrMalformed)
{
  configure();
  auto bad = message(0.0);
  bad.joint_names = {"b", "c"};  // misses goal joint 'a'
  for (int i = 0; i < 40; ++i) {
    pub->publish(bad);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  term.compute(state, TensorMap{}, out.data());
  expect_out(kHold);

  stream(message(0.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // older than the 0.4 s timeout
  term.compute(state, TensorMap{}, out.data());
  expect_out(kHold);
}

TEST_F(MotionGoalTermTest, HoldsWhileTheSafetyEntryIsZero)
{
  state.add("safety/safe");
  configure(1, "safety/safe");
  state.set(state.index("safety/safe"), 1.0, 1.0);
  stream(message(0.0));
  state.set(state.index("safety/safe"), 0.0, 1.0);
  term.compute(state, TensorMap{}, out.data());
  expect_out(kHold);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
