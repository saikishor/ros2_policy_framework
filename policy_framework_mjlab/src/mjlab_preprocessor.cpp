// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework_mjlab/mjlab_preprocessor.hpp"

#include <algorithm>
#include <memory>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "mjlab_preprocessor_parameters.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "policy_framework_mjlab/metadata.hpp"

using policy_framework::DataStore;
using policy_framework::get_parameter;
using policy_framework::TensorMap;

namespace policy_framework_mjlab
{

void projected_gravity(double w, double x, double y, double z, float out[3])
{
  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  w /= n;
  // Rotate v = (0, 0, -1) by the conjugate quaternion (w, -x, -y, -z):
  // v' = v + w t + r x t, with r = (-x, -y, -z) and t = 2 r x v.
  const double rx = -x / n, ry = -y / n, rz = -z / n;
  const double tx = -2.0 * ry, ty = 2.0 * rx, tz = 0.0;  // 2 r x (0, 0, -1)
  out[0] = static_cast<float>(w * tx + (ry * tz - rz * ty));
  out[1] = static_cast<float>(w * ty + (rz * tx - rx * tz));
  out[2] = static_cast<float>(-1.0 + w * tz + (rx * ty - ry * tx));
}

void MjlabPreprocessor::configure(
  const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
  DataStore & state)
{
  const auto & md = model.metadata;
  const auto joints = split(require(md, "joint_names"));
  default_joint_pos_ = parse_doubles(require(md, "default_joint_pos"));
  if (default_joint_pos_.size() != joints.size()) {
    throw std::runtime_error("default_joint_pos and joint_names sizes differ");
  }
  const auto cmd_it = md.find("command_names");
  const bool motion = cmd_it != md.end() && cmd_it->second == "motion";

  const auto params = mjlab_preprocessor::ParamListener(ctx.node, ctx.name).get_params();
  const auto & imu = params.imu_name;
  const auto & cmd = params.velocity_command_name;
  const auto & odom = params.odometry_name;
  obs_input_ = params.observation_input;

  required_.clear();
  auto add = [&](const std::string & name) {
      const auto i = state.add(name);
      if (std::find(required_.begin(), required_.end(), i) == required_.end()) {
        required_.push_back(i);
      }
      return i;
    };
  auto add_all = [&](const std::string & prefix, const std::vector<std::string> & fields) {
      std::vector<size_t> idx;
      for (const auto & f : fields) {idx.push_back(add(prefix + f));}
      return idx;
    };
  auto output_size = [&](const std::string & name) {
      const auto it = model.outputs.find(name);
      if (it == model.outputs.end()) {
        throw std::runtime_error("Model has no '" + name + "' output");
      }
      return it->second.data.size();
    };
  auto joint_entries = [&](const std::string & iface) {
      std::vector<size_t> idx;
      for (const auto & j : joints) {idx.push_back(add(j + "/" + iface));}
      return idx;
    };

  terms_.clear();
  stamps_.clear();
  size_t single_dim = 0;
  for (const auto & name : split(require(md, "observation_names"))) {
    Term t{};
    // Not a generated parameter: the term names come from the model metadata.
    const auto plugin = get_parameter<std::string>(ctx, "terms." + name + ".type", "");
    if (!plugin.empty()) {
      if (!term_loader_) {
        term_loader_ = std::make_unique<pluginlib::ClassLoader<policy_framework::ObservationTerm>>(
          "policy_framework", "policy_framework::ObservationTerm");
      }
      t.kind = Kind::PLUGIN;
      t.plugin = term_loader_->createSharedInstance(plugin);
      t.dim = t.plugin->configure(
        {ctx.node, ctx.name + ".terms." + name, ctx.kinematics}, model, state);
      if (t.dim == 0) {
        throw std::runtime_error("Observation term plugin '" + name + "' has size 0");
      }
    } else if (name == "base_ang_vel") {
      t = {Kind::BASE_ANG_VEL, 3, {}, nullptr};
      ang_vel_ = add_all(imu + "/angular_velocity.", {"x", "y", "z"});
    } else if (name == "base_lin_acc") {
      t = {Kind::BASE_LIN_ACC, 3, {}, nullptr};
      lin_acc_ = add_all(imu + "/linear_acceleration.", {"x", "y", "z"});
    } else if (name == "imu_projected_gravity" || name == "projected_gravity") {
      t = {Kind::PROJECTED_GRAVITY, 3, {}, nullptr};
      quat_ = add_all(imu + "/orientation.", {"w", "x", "y", "z"});
    } else if (name == "base_lin_vel") {
      t = {Kind::BASE_LIN_VEL, 3, {}, nullptr};
      lin_vel_ = add_all(odom + "/linear.", {"x", "y", "z"});
    } else if (name == "joint_pos") {
      t = {Kind::JOINT_POS, joints.size(), {}, nullptr};
      joint_pos_ = joint_entries("position");
    } else if (name == "joint_vel") {
      t = {Kind::JOINT_VEL, joints.size(), {}, nullptr};
      joint_vel_ = joint_entries("velocity");
    } else if (name == "actions") {
      t = {Kind::ACTIONS, output_size("actions"), {}, nullptr};
    } else if (name == "command" && motion) {
      if (output_size("joint_pos") != joints.size() || output_size("joint_vel") != joints.size()) {
        throw std::runtime_error("Motion outputs joint_pos/joint_vel must have one value per joint");
      }
      t = {Kind::MOTION_COMMAND, 2 * joints.size(), {}, nullptr};
      // Seeds the command on the first step after reset.
      joint_pos_ = joint_entries("position");
      joint_vel_ = joint_entries("velocity");
    } else if (name == "command") {
      t = {Kind::TWIST_COMMAND, 3, {}, nullptr};
      twist_ = add_all(cmd + "/", {"linear.x", "linear.y", "angular.z"});
    } else {
      throw std::runtime_error("Unsupported mjlab observation term '" + name + "'");
    }
    single_dim += t.dim;
    terms_.push_back(std::move(t));
    stamps_.push_back({name, 0.0});
  }

  const auto in = model.inputs.find(obs_input_);
  if (in == model.inputs.end()) {
    throw std::runtime_error("Model has no '" + obs_input_ + "' input");
  }
  const auto obs_dim = in->second.data.size();
  if (single_dim == 0 || obs_dim % single_dim != 0) {
    throw std::runtime_error(
            "Observation size " + std::to_string(obs_dim) +
            " is not a multiple of the single observation size " + std::to_string(single_dim));
  }
  history_length_ = obs_dim / single_dim;
  has_time_step_ = model.inputs.count("time_step") > 0;
  for (auto & t : terms_) {t.history.assign(history_length_ * t.dim, 0.0F);}
  scratch_.assign(single_dim, 0.0F);
  RCLCPP_INFO(
    ctx.node->get_logger(), "[%s] mjlab observation: %zu terms, single size %zu, history %zu%s",
    ctx.name.c_str(), terms_.size(), single_dim, history_length_,
    motion ? ", motion command" : "");
}

void MjlabPreprocessor::clear()
{
  history_filled_ = false;
  prev_obs_stamp_ = 0.0;
}

void MjlabPreprocessor::reset()
{
  first_step_ = true;
  step_ = 0;
  for (auto & t : terms_) {
    if (t.plugin) {t.plugin->reset();}
  }
}

double MjlabPreprocessor::current_value(
  const Term & term, const DataStore & state, const TensorMap & last_outputs, float * out) const
{
  auto oldest = [&](const std::vector<size_t> & idx) {
      double t = std::numeric_limits<double>::max();
      for (const auto i : idx) {t = std::min(t, state.stamp(i));}
      return t;
    };
  // Copy the entries and return their oldest stamp.
  auto copy = [&](const std::vector<size_t> & idx, float * dst) {
      for (size_t i = 0; i < idx.size(); ++i) {dst[i] = static_cast<float>(state.get(idx[i]));}
      return oldest(idx);
    };
  switch (term.kind) {
    case Kind::BASE_ANG_VEL: return copy(ang_vel_, out);
    case Kind::BASE_LIN_ACC: return copy(lin_acc_, out);
    case Kind::BASE_LIN_VEL: return copy(lin_vel_, out);
    case Kind::TWIST_COMMAND: return copy(twist_, out);
    case Kind::JOINT_VEL: return copy(joint_vel_, out);
    case Kind::PLUGIN: return term.plugin->compute(state, last_outputs, out);
    case Kind::PROJECTED_GRAVITY:
      projected_gravity(
        state.get(quat_[0]), state.get(quat_[1]), state.get(quat_[2]), state.get(quat_[3]), out);
      return oldest(quat_);
    case Kind::JOINT_POS:
      for (size_t i = 0; i < joint_pos_.size(); ++i) {
        out[i] = static_cast<float>(state.get(joint_pos_[i]) - default_joint_pos_[i]);
      }
      return oldest(joint_pos_);
    case Kind::ACTIONS: {
        const auto & a = last_outputs.at("actions").data;
        std::copy(a.begin(), a.end(), out);
        return prev_obs_stamp_;
      }
    case Kind::MOTION_COMMAND: {
        const size_t n = joint_pos_.size();
        if (first_step_) {
          // Start the reference motion from where the robot is.
          return std::min(copy(joint_pos_, out), copy(joint_vel_, out + n));
        }
        const auto & p = last_outputs.at("joint_pos").data;
        const auto & v = last_outputs.at("joint_vel").data;
        std::copy(p.begin(), p.end(), out);
        std::copy(v.begin(), v.end(), out + n);
        return prev_obs_stamp_;
      }
  }
  return 0.0;
}

bool MjlabPreprocessor::compute(
  const DataStore & state, const TensorMap & last_outputs, TensorMap & inputs)
{
  if (!state.all_received(required_)) {
    return false;
  }
  for (auto & t : terms_) {
    if (t.plugin && !t.plugin->ready(state)) {return false;}
  }
  auto & obs = inputs.at(obs_input_).data;
  size_t offset = 0;
  double obs_stamp = 0.0;  // oldest known stamp of the terms not derived from previous outputs
  for (size_t k = 0; k < terms_.size(); ++k) {
    auto & t = terms_[k];
    // Drop the oldest entry, append the current value as the newest.
    std::copy(t.history.begin() + t.dim, t.history.end(), t.history.begin());
    float * newest = t.history.data() + t.history.size() - t.dim;
    const double stamp = current_value(t, state, last_outputs, newest);
    const bool derived = t.kind == Kind::ACTIONS ||
      (t.kind == Kind::MOTION_COMMAND && !first_step_);
    stamps_[k].stamp = stamp;
    stamps_[k].from_previous_outputs = derived;
    if (!derived && stamp > 0.0 && (obs_stamp <= 0.0 || stamp < obs_stamp)) {
      obs_stamp = stamp;
    }
    if (!history_filled_) {
      // Like mjlab's CircularBuffer: the first push after clear back-fills the whole history.
      for (size_t h = 0; h + 1 < history_length_; ++h) {
        std::copy(newest, newest + t.dim, t.history.begin() + h * t.dim);
      }
    }
    std::copy(t.history.begin(), t.history.end(), obs.begin() + offset);
    offset += t.history.size();
  }
  if (has_time_step_) {
    inputs.at("time_step").data[0] = static_cast<float>(step_);
  }
  history_filled_ = true;
  prev_obs_stamp_ = obs_stamp;
  first_step_ = false;
  ++step_;
  return true;
}

}  // namespace policy_framework_mjlab

PLUGINLIB_EXPORT_CLASS(policy_framework_mjlab::MjlabPreprocessor, policy_framework::Preprocessor)
