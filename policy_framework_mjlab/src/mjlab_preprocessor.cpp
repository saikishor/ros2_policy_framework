// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework_mjlab/mjlab_preprocessor.hpp"

#include <algorithm>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "policy_framework_mjlab/mjlab_preprocessor_parameters.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "policy_framework_mjlab/metadata.hpp"

using policy_framework::DataStore;
using policy_framework::get_parameter;
using policy_framework::TensorMap;

namespace policy_framework_mjlab
{

void projected_gravity(double w, double x, double y, double z, float out[3])
{
  // R^T (0, 0, -1): the negated last row of the rotation matrix of q / |q|.
  const double s = 2.0 / (w * w + x * x + y * y + z * z);
  out[0] = static_cast<float>(s * (w * y - x * z));
  out[1] = static_cast<float>(-s * (w * x + y * z));
  out[2] = static_cast<float>(s * (x * x + y * y) - 1.0);
}

void MjlabPreprocessor::configure(
  const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
  DataStore & state)
{
  const auto & md = model.metadata;
  auto [joints, defaults] = policy_framework_mjlab::joints(md);
  default_joint_pos_ = std::move(defaults);
  const auto cmd_it = md.find("command_names");
  const bool motion = cmd_it != md.end() && cmd_it->second == "motion";

  const auto params = mjlab_preprocessor::ParamListener(ctx.node, ctx.name).get_params();
  const auto & imu = params.imu_name;
  const auto & cmd = params.velocity_command_name;
  const auto & odom = params.odometry_name;
  obs_input_ = params.observation_input;

  required_.clear();
  // Register prefix + name + suffix for every name; duplicates in required_ are harmless.
  auto add = [&](const std::string & prefix, const std::vector<std::string> & names,
      const std::string & suffix = "") {
      std::vector<size_t> idx;
      for (const auto & n : names) {idx.push_back(state.add(prefix + n + suffix));}
      required_.insert(required_.end(), idx.begin(), idx.end());
      return idx;
    };
  auto output_size = [&](const std::string & name) {
      const auto it = model.outputs.find(name);
      if (it == model.outputs.end()) {
        throw std::runtime_error("Model has no '" + name + "' output");
      }
      return it->second.data.size();
    };
  const std::vector<std::string> xyz{"x", "y", "z"};

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
      t = {Kind::STATE, 3, add(imu + "/angular_velocity.", xyz)};
    } else if (name == "base_lin_acc") {
      t = {Kind::STATE, 3, add(imu + "/linear_acceleration.", xyz)};
    } else if (name == "imu_projected_gravity" || name == "projected_gravity") {
      t = {Kind::PROJECTED_GRAVITY, 3, add(imu + "/orientation.", {"w", "x", "y", "z"})};
    } else if (name == "base_lin_vel") {
      t = {Kind::STATE, 3, add(odom + "/linear.", xyz)};
    } else if (name == "joint_pos") {
      t = {Kind::JOINT_POS, joints.size(), add("", joints, "/position")};
    } else if (name == "joint_vel") {
      t = {Kind::STATE, joints.size(), add("", joints, "/velocity")};
    } else if (name == "actions") {
      t = {Kind::ACTIONS, output_size("actions")};
    } else if (name == "command" && motion) {
      if (output_size("joint_pos") != joints.size() || output_size("joint_vel") != joints.size()) {
        throw std::runtime_error("Motion outputs joint_pos/joint_vel must have one value per joint");
      }
      // Positions then velocities: they seed the command on the first step after reset.
      t = {Kind::MOTION_COMMAND, 2 * joints.size(), add("", joints, "/position")};
      const auto vel = add("", joints, "/velocity");
      t.idx.insert(t.idx.end(), vel.begin(), vel.end());
    } else if (name == "command") {
      t = {Kind::STATE, 3, add(cmd + "/", {"linear.x", "linear.y", "angular.z"})};
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
  const auto & idx = term.idx;
  switch (term.kind) {
    case Kind::STATE: return copy(idx, out);
    case Kind::PLUGIN: return term.plugin->compute(state, last_outputs, out);
    case Kind::PROJECTED_GRAVITY:
      projected_gravity(
        state.get(idx[0]), state.get(idx[1]), state.get(idx[2]), state.get(idx[3]), out);
      return oldest(idx);
    case Kind::JOINT_POS:
      for (size_t i = 0; i < idx.size(); ++i) {
        out[i] = static_cast<float>(state.get(idx[i]) - default_joint_pos_[i]);
      }
      return oldest(idx);
    case Kind::ACTIONS: {
        const auto & a = last_outputs.at("actions").data;
        std::copy(a.begin(), a.end(), out);
        return prev_obs_stamp_;
      }
    case Kind::MOTION_COMMAND: {
        if (step_ == 0) {return copy(idx, out);}  // start the reference motion from the robot
        const auto & p = last_outputs.at("joint_pos").data;
        const auto & v = last_outputs.at("joint_vel").data;
        std::copy(v.begin(), v.end(), std::copy(p.begin(), p.end(), out));
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
      (t.kind == Kind::MOTION_COMMAND && step_ > 0);
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
  ++step_;
  return true;
}

}  // namespace policy_framework_mjlab

PLUGINLIB_EXPORT_CLASS(policy_framework_mjlab::MjlabPreprocessor, policy_framework::Preprocessor)
