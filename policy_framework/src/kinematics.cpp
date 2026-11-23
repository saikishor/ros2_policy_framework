// Copyright 2026 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "policy_framework/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace policy_framework
{

struct Kinematics::Model
{
  pinocchio::Model model;
  pinocchio::Data data;
  std::vector<std::string> joint_names;
  /// Per exposed joint: start in Pinocchio's q and v, and whether q holds (cos, sin).
  std::vector<Eigen::Index> idx_q;
  std::vector<Eigen::Index> idx_v;
  std::vector<bool> unbounded;
  Eigen::VectorXd q;  ///< Pinocchio configuration (nq)
  Eigen::VectorXd v;  ///< Pinocchio velocity (nv)
  bool ready = false;
  double stamp = 0.0;
};

namespace
{

using Lock = std::lock_guard<realtime_tools::prio_inherit_recursive_mutex>;

Eigen::Isometry3d to_isometry(const pinocchio::SE3 & t)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = t.rotation();
  out.translation() = t.translation();
  return out;
}

}  // namespace

Kinematics::Kinematics(const std::string & robot_description, const std::vector<std::string> & joints)
: model_(std::make_unique<Model>())
{
  if (robot_description.empty()) {throw std::runtime_error("Kinematics: empty robot description");}
  pinocchio::Model full;
  try {
    pinocchio::urdf::buildModelFromXML(robot_description, full);
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Kinematics: cannot parse the robot description: ") + e.what());
  }
  auto & m = *model_;
  if (joints.empty()) {
    m.model = full;
  } else {
    for (const auto & j : joints) {
      if (!full.existJointName(j)) {throw std::runtime_error("Kinematics: unknown joint '" + j + "'");}
    }
    std::vector<pinocchio::JointIndex> locked;
    for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(full.njoints); ++j) {
      if (std::find(joints.begin(), joints.end(), full.names[j]) == joints.end()) {
        locked.push_back(j);
      }
    }
    // Locked at the neutral configuration: zero angle / displacement.
    m.model = pinocchio::buildReducedModel(full, locked, pinocchio::neutral(full));
  }
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(m.model.njoints); ++j) {
    const int nq = m.model.nqs[j];
    if (m.model.nvs[j] != 1 || (nq != 1 && nq != 2)) {
      throw std::runtime_error(
              "Kinematics: joint '" + m.model.names[j] +
              "' is not revolute, continuous or prismatic: leave it out of the joints");
    }
    m.joint_names.push_back(m.model.names[j]);
    m.idx_q.push_back(m.model.idx_qs[j]);
    m.idx_v.push_back(m.model.idx_vs[j]);
    m.unbounded.push_back(nq == 2);  // continuous joint: (cos, sin)
  }
  m.data = pinocchio::Data(m.model);
  m.q = pinocchio::neutral(m.model);
  m.v = Eigen::VectorXd::Zero(m.model.nv);
}

Kinematics::~Kinematics() = default;

const std::vector<std::string> & Kinematics::joint_names() const {return model_->joint_names;}

size_t Kinematics::frame(const std::string & link) const
{
  // Links only (BODY frames), so that a joint of the same name cannot match.
  if (!model_->model.existFrame(link, pinocchio::BODY)) {
    throw std::runtime_error("Kinematics: unknown link '" + link + "'");
  }
  return model_->model.getFrameId(link, pinocchio::BODY);
}

bool Kinematics::update(const Eigen::VectorXd & q, const Eigen::VectorXd & v, double stamp)
{
  auto & m = *model_;
  const auto n = static_cast<Eigen::Index>(m.joint_names.size());
  if (q.size() != n || v.size() != n) {
    throw std::invalid_argument("Kinematics::update: q and v need one value per joint_names()");
  }
  std::unique_lock<realtime_tools::prio_inherit_recursive_mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {return false;}
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto j = static_cast<size_t>(i);
    if (m.unbounded[j]) {
      m.q[m.idx_q[j]] = std::cos(q[i]);
      m.q[m.idx_q[j] + 1] = std::sin(q[i]);
    } else {
      m.q[m.idx_q[j]] = q[i];
    }
    m.v[m.idx_v[j]] = v[i];
  }
  pinocchio::forwardKinematics(m.model, m.data, m.q, m.v);
  pinocchio::updateFramePlacements(m.model, m.data);
  m.ready = true;
  m.stamp = stamp;
  return true;
}

bool Kinematics::ready() const
{
  Lock lock(mutex_);
  return model_->ready;
}

double Kinematics::stamp() const
{
  Lock lock(mutex_);
  return model_->stamp;
}

Eigen::Isometry3d Kinematics::transform(size_t frame) const
{
  Lock lock(mutex_);
  return to_isometry(model_->data.oMf[frame]);
}

Eigen::Isometry3d Kinematics::transform(size_t frame, size_t reference) const
{
  Lock lock(mutex_);
  return to_isometry(model_->data.oMf[reference].actInv(model_->data.oMf[frame]));
}

Eigen::Matrix<double, 6, 1> Kinematics::velocity(size_t frame, Reference reference) const
{
  Lock lock(mutex_);
  return pinocchio::getFrameVelocity(
    model_->model, model_->data, frame,
    reference == Reference::LOCAL ? pinocchio::LOCAL : pinocchio::LOCAL_WORLD_ALIGNED).toVector();
}

Eigen::Quaterniond Kinematics::world_orientation(
  size_t frame, size_t imu_frame, const Eigen::Quaterniond & imu) const
{
  Lock lock(mutex_);
  const auto & o = model_->data.oMf;
  // R_world_frame = R_world_imu * R_root_imu^T * R_root_frame
  return imu * Eigen::Quaterniond(o[imu_frame].rotation().transpose() * o[frame].rotation());
}

Eigen::Vector3d Kinematics::world_angular_velocity(
  size_t frame, size_t imu_frame, const Eigen::Vector3d & imu_angular_velocity) const
{
  Lock lock(mutex_);
  const auto & m = *model_;
  // Of the frame w.r.t. the root, in the frame's axes. A copy: angular() refers to a temporary.
  auto relative = [&](size_t f) -> Eigen::Vector3d {
      return pinocchio::getFrameVelocity(m.model, m.data, f, pinocchio::LOCAL).angular();
    };
  // The root turns as the IMU does minus what the joints add; the frame as the root plus what
  // the joints add to it.
  const Eigen::Vector3d root_in_imu = imu_angular_velocity - relative(imu_frame);
  const Eigen::Matrix3d frame_from_imu =
    m.data.oMf[frame].rotation().transpose() * m.data.oMf[imu_frame].rotation();
  return frame_from_imu * root_in_imu + relative(frame);
}

}  // namespace policy_framework
