// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/kinematics.hpp"

#include <algorithm>
#include <stdexcept>

#include "kinematics_parameters.hpp"
#include "urdf/model.h"

namespace policy_framework
{
namespace
{

/**
 * @return Movable joints from @p base to @p tip, in the order of a kinematics_interface joint
 *         vector (base to tip; fixed, floating and planar joints are not part of it).
 */
std::vector<std::string> chain_joints(
  const std::string & robot_description, const std::string & base, const std::string & tip)
{
  urdf::Model model;
  if (!model.initString(robot_description)) {
    throw std::runtime_error("Kinematics: cannot parse the robot description");
  }
  auto link = model.getLink(tip);
  if (!link) {throw std::runtime_error("Kinematics: unknown tip link '" + tip + "'");}
  std::vector<std::string> joints;
  while (link->name != base) {
    const auto joint = link->parent_joint;
    if (!joint) {
      throw std::runtime_error(
              "Kinematics: base '" + base + "' is not an ancestor of tip '" + tip + "'");
    }
    if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS ||
      joint->type == urdf::Joint::PRISMATIC)
    {
      joints.push_back(joint->name);
    }
    link = model.getLink(joint->parent_link_name);
  }
  std::reverse(joints.begin(), joints.end());
  return joints;
}

}  // namespace

std::shared_ptr<Kinematics> Kinematics::create(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & robot_description, DataStore & state)
{
  const auto params = kinematics::ParamListener(node, "kinematics").get_params();
  if (params.plugin_name.empty()) {return nullptr;}
  if (robot_description.empty()) {
    throw std::runtime_error("Kinematics: '" + params.plugin_name + "' needs a robot description");
  }
  if (params.base.empty() || params.tip.empty()) {
    throw std::runtime_error("Kinematics: kinematics.base and kinematics.tip are required");
  }
  if (params.imu_sensor.empty() != params.imu_link.empty()) {
    throw std::runtime_error("Kinematics: set both kinematics.imu_sensor and imu_link, or none");
  }

  std::shared_ptr<Kinematics> kin(new Kinematics());
  kin->joint_names_ = chain_joints(robot_description, params.base, params.tip);
  kin->loader_ = std::make_unique<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
    "kinematics_interface", "kinematics_interface::KinematicsInterface");
  kin->plugin_ = kin->loader_->createSharedInstance(params.plugin_name);
  // The plugin reads kinematics.base / tip / alpha, declared above by the ParamListener.
  if (!kin->plugin_->initialize(
      robot_description, node->get_node_parameters_interface(), "kinematics"))
  {
    throw std::runtime_error("Kinematics: '" + params.plugin_name + "' failed to initialize");
  }
  kin->q_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(kin->joint_names_.size()));
  for (const auto & j : kin->joint_names_) {kin->joint_idx_.push_back(state.add(j + "/position"));}
  if (!params.imu_sensor.empty()) {
    kin->imu_frame_ = kin->frame(params.imu_link);
    for (const auto * c : {"x", "y", "z", "w"}) {
      kin->imu_idx_.push_back(state.add(params.imu_sensor + "/orientation." + c));
    }
  }
  return kin;
}

Kinematics::~Kinematics()
{
  plugin_.reset();  // before loader_ unloads its library
}

size_t Kinematics::frame(const std::string & link)
{
  const auto it = std::find(links_.begin(), links_.end(), link);
  if (it != links_.end()) {return static_cast<size_t>(it - links_.begin());}
  Eigen::Isometry3d t;
  if (!plugin_->calculate_link_transform(q_, link, t)) {
    throw std::runtime_error("Kinematics: link '" + link + "' is not on the chain");
  }
  links_.push_back(link);
  transforms_.push_back(t);
  return links_.size() - 1;
}

void Kinematics::update(const DataStore & state)
{
  ready_ = state.all_received(joint_idx_) && state.all_received(imu_idx_);
  if (!ready_) {return;}
  stamp_ = 0.0;
  const auto track_stamp = [&](size_t i) {
      if (stamp_ <= 0.0 || state.stamp(i) < stamp_) {stamp_ = state.stamp(i);}
    };
  for (size_t j = 0; j < joint_idx_.size(); ++j) {
    q_[static_cast<Eigen::Index>(j)] = state.get(joint_idx_[j]);
    track_stamp(joint_idx_[j]);
  }
  for (size_t f = 0; f < links_.size(); ++f) {
    ready_ = plugin_->calculate_link_transform(q_, links_[f], transforms_[f]) && ready_;
  }
  if (!imu_idx_.empty()) {
    imu_orientation_ = Eigen::Quaterniond(
      state.get(imu_idx_[3]), state.get(imu_idx_[0]), state.get(imu_idx_[1]),
      state.get(imu_idx_[2])).normalized();
    track_stamp(imu_idx_[0]);
  }
}

Eigen::Quaterniond Kinematics::world_orientation(size_t frame) const
{
  if (imu_idx_.empty()) {
    throw std::logic_error("Kinematics: world_orientation() needs kinematics.imu_sensor");
  }
  // R_world_frame = R_world_imu * R_base_imu^T * R_base_frame
  return imu_orientation_ *
         Eigen::Quaterniond(transforms_[imu_frame_].linear()).conjugate() *
         Eigen::Quaterniond(transforms_[frame].linear());
}

}  // namespace policy_framework
