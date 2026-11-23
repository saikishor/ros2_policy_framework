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
#include "policy_framework_mjlab/base_from_imu.hpp"
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

namespace
{

using policy_framework::ModelInfo;
using policy_framework::ObservationTerm;
using policy_framework::PluginContext;
using Names = std::vector<std::string>;

/// prefix + name + suffix for every name.
Names entries(const std::string & prefix, const Names & names, const std::string & suffix = "")
{
  Names out;
  for (const auto & n : names) {out.push_back(prefix + n + suffix);}
  return out;
}

size_t output_size(const ModelInfo & model, const std::string & name)
{
  const auto it = model.outputs.find(name);
  if (it == model.outputs.end()) {throw std::runtime_error("Model has no '" + name + "' output");}
  return it->second.data.size();
}

/// State entries as they are, minus an optional offset per entry.
class StateTerm : public ObservationTerm
{
public:
  explicit StateTerm(Names names, std::vector<double> offsets = {})
  : names_(std::move(names)), offsets_(std::move(offsets)) {}

  size_t configure(const PluginContext &, const ModelInfo &, DataStore & state) override
  {
    idx_.clear();
    for (const auto & n : names_) {idx_.push_back(state.add(n));}
    offsets_.resize(idx_.size(), 0.0);
    return idx_.size();
  }
  bool ready(const DataStore & state) override {return state.all_received(idx_);}
  double compute(const DataStore & state, const TensorMap &, float * out) override
  {
    for (size_t i = 0; i < idx_.size(); ++i) {
      out[i] = static_cast<float>(state.get(idx_[i]) - offsets_[i]);
    }
    return oldest_stamp(state);
  }

protected:
  double oldest_stamp(const DataStore & state) const
  {
    double t = std::numeric_limits<double>::max();
    for (const auto i : idx_) {t = std::min(t, state.stamp(i));}
    return t;
  }
  std::vector<size_t> idx_;

private:
  Names names_;
  std::vector<double> offsets_;
};

/// Gravity in the frame of the orientation entries `<prefix>{w,x,y,z}`.
class ProjectedGravityTerm : public StateTerm
{
public:
  explicit ProjectedGravityTerm(const std::string & prefix)
  : StateTerm(entries(prefix, {"w", "x", "y", "z"})) {}

  size_t configure(const PluginContext & ctx, const ModelInfo & model, DataStore & state) override
  {
    StateTerm::configure(ctx, model, state);
    return 3;
  }
  double compute(const DataStore & state, const TensorMap &, float * out) override
  {
    projected_gravity(
      state.get(idx_[0]), state.get(idx_[1]), state.get(idx_[2]), state.get(idx_[3]), out);
    return oldest_stamp(state);
  }
};

/// The raw `actions` output of the previous step.
class ActionsTerm : public ObservationTerm
{
public:
  size_t configure(const PluginContext &, const ModelInfo & model, DataStore &) override
  {
    return output_size(model, "actions");
  }
  bool ready(const DataStore &) override {return true;}
  double compute(const DataStore &, const TensorMap & last_outputs, float * out) override
  {
    const auto & a = last_outputs.at("actions").data;
    std::copy(a.begin(), a.end(), out);
    return kFromPreviousOutputs;
  }
};

/// Motion command: the `joint_pos`, `joint_vel` outputs of the previous step; the robot's joint
/// positions and velocities on the first step after reset(), to start the motion from there.
class MotionCommandTerm : public StateTerm
{
public:
  explicit MotionCommandTerm(const Names & joints)
  : StateTerm(
      [&]() {
        auto names = entries("", joints, "/position");
        const auto vel = entries("", joints, "/velocity");
        names.insert(names.end(), vel.begin(), vel.end());
        return names;
      }()),
    joints_(joints.size()) {}

  size_t configure(const PluginContext & ctx, const ModelInfo & model, DataStore & state) override
  {
    if (output_size(model, "joint_pos") != joints_ || output_size(model, "joint_vel") != joints_) {
      throw std::runtime_error("Motion outputs joint_pos/joint_vel must have one value per joint");
    }
    return StateTerm::configure(ctx, model, state);
  }
  void reset() override {first_ = true;}
  double compute(const DataStore & state, const TensorMap & last_outputs, float * out) override
  {
    if (std::exchange(first_, false)) {return StateTerm::compute(state, last_outputs, out);}
    const auto & p = last_outputs.at("joint_pos").data;
    const auto & v = last_outputs.at("joint_vel").data;
    std::copy(v.begin(), v.end(), std::copy(p.begin(), p.end(), out));
    return kFromPreviousOutputs;
  }

private:
  size_t joints_;
  bool first_ = true;
};

/// pal_mjlab base terms: the IMU measurements moved to the base link with the kinematics.
class BaseTerm : public ObservationTerm
{
public:
  enum class Kind {ANGULAR_VELOCITY, PROJECTED_GRAVITY};
  BaseTerm(Kind kind, std::string imu_name, std::string imu_frame, std::string base)
  : kind_(kind), imu_name_(std::move(imu_name)), imu_frame_(std::move(imu_frame)),
    base_body_(std::move(base)) {}

  size_t configure(const PluginContext & ctx, const ModelInfo &, DataStore & state) override
  {
    base_.configure(ctx, state, imu_name_, imu_frame_, base_body_);
    return 3;
  }
  bool ready(const DataStore & state) override {return base_.ready(state);}
  double compute(const DataStore & state, const TensorMap &, float * out) override
  {
    if (kind_ == Kind::ANGULAR_VELOCITY) {
      const Eigen::Vector3d w = base_.angular_velocity(state);
      for (int i = 0; i < 3; ++i) {out[i] = static_cast<float>(w[i]);}
    } else {
      const Eigen::Quaterniond q = base_.orientation(state);
      projected_gravity(q.w(), q.x(), q.y(), q.z(), out);
    }
    return base_.stamp(state);
  }

private:
  Kind kind_;
  std::string imu_name_, imu_frame_, base_body_;
  BaseFromImu base_;
};

}  // namespace

void MjlabPreprocessor::configure(
  const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
  DataStore & state)
{
  const auto & md = model.metadata;
  const auto [joints, defaults] = policy_framework_mjlab::joints(md, "observation_joint_names");
  const auto cmd_it = md.find("command_names");
  const bool motion = cmd_it != md.end() && cmd_it->second == "motion";

  const auto params = mjlab_preprocessor::ParamListener(ctx.node, ctx.name).get_params();
  const auto & imu = params.imu_name;
  obs_input_ = params.observation_input;

  // pal_mjlab exports that observe their base name its link; the IMU may sit on another one.
  const auto base_it = md.find("goal_root_body");
  const std::string base = base_it == md.end() ? "" : base_it->second;
  const Names xyz{"x", "y", "z"};
  // The built-in term of each name; null for a name this preprocessor does not know.
  auto builtin = [&](const std::string & name) -> std::shared_ptr<ObservationTerm> {
      if (name == "base_ang_vel" && !base.empty()) {
        return std::make_shared<BaseTerm>(
          BaseTerm::Kind::ANGULAR_VELOCITY, imu, params.imu_frame, base);
      }
      if (name == "base_projected_gravity") {
        return std::make_shared<BaseTerm>(
          BaseTerm::Kind::PROJECTED_GRAVITY, imu, params.imu_frame, require(md, "goal_root_body"));
      }
      if (name == "base_ang_vel" || name == "imu_ang_vel") {
        return std::make_shared<StateTerm>(entries(imu + "/angular_velocity.", xyz));
      }
      if (name == "base_lin_acc") {
        return std::make_shared<StateTerm>(entries(imu + "/linear_acceleration.", xyz));
      }
      if (name == "imu_projected_gravity" || name == "projected_gravity") {
        return std::make_shared<ProjectedGravityTerm>(imu + "/orientation.");
      }
      if (name == "base_lin_vel") {
        return std::make_shared<StateTerm>(entries(params.odometry_name + "/linear.", xyz));
      }
      if (name == "joint_pos") {
        return std::make_shared<StateTerm>(entries("", joints, "/position"), defaults);
      }
      if (name == "joint_vel") {return std::make_shared<StateTerm>(entries("", joints, "/velocity"));}
      if (name == "actions") {return std::make_shared<ActionsTerm>();}
      if (name == "command" && motion) {return std::make_shared<MotionCommandTerm>(joints);}
      if (name == "command" || name == "velocity_command") {
        return std::make_shared<StateTerm>(
          entries(params.velocity_command_name + "/", {"linear.x", "linear.y", "angular.z"}));
      }
      return nullptr;
    };

  const auto names = split(require(md, "observation_names"));
  terms_.clear();
  stamps_.clear();
  size_t single_dim = 0;
  for (const auto & name : names) {
    Term t;
    // Not a generated parameter: the term names come from the model metadata.
    const auto plugin = get_parameter<std::string>(ctx, "terms." + name + ".type", "");
    if (!plugin.empty()) {
      if (!term_loader_) {
        term_loader_ = std::make_unique<pluginlib::ClassLoader<ObservationTerm>>(
          "policy_framework", "policy_framework::ObservationTerm");
      }
      t.term = term_loader_->createSharedInstance(plugin);
    } else {
      t.term = builtin(name);
    }
    if (!t.term) {throw std::runtime_error("Unsupported mjlab observation term '" + name + "'");}
    t.dim = t.term->configure(
      {ctx.node, ctx.name + ".terms." + name, ctx.kinematics, ctx.safety_entry}, model, state);
    if (t.dim == 0) {
      throw std::runtime_error("Observation term '" + name + "' has size 0");
    }
    single_dim += t.dim;
    terms_.push_back(std::move(t));
    const auto & term = *terms_.back().term;
    stamps_.push_back(
      {name, 0.0, false, term.user_command(), 0.0, term.max_age(), term.max_age_on_fault()});
  }

  const auto in = model.inputs.find(obs_input_);
  if (in == model.inputs.end()) {
    throw std::runtime_error("Model has no '" + obs_input_ + "' input");
  }
  const auto obs_dim = in->second.data.size();
  if (md.count("observation_terms_dim")) {
    const auto dims = parse_counts(md, "observation_terms_dim", names.size());
    for (size_t k = 0; k < names.size(); ++k) {
      if (dims[k] != terms_[k].dim) {
        throw std::runtime_error(
                "Observation term '" + names[k] + "' has size " + std::to_string(dims[k]) +
                " in the model metadata but " + std::to_string(terms_[k].dim) + " here");
      }
    }
  }
  std::vector<size_t> lengths;
  if (md.count("observation_terms_history_length")) {
    // pal_mjlab exports: a history length per term.
    lengths = parse_counts(md, "observation_terms_history_length", names.size(), 0);
    // mjlab's history_length 0 means "no history": the term still feeds its current frame.
    for (auto & l : lengths) {l = std::max<size_t>(l, 1);}
  } else {
    if (single_dim == 0 || obs_dim % single_dim != 0) {
      throw std::runtime_error(
              "Observation size " + std::to_string(obs_dim) +
              " is not a multiple of the single observation size " + std::to_string(single_dim));
    }
    lengths.assign(names.size(), obs_dim / single_dim);
  }
  size_t total = 0;
  history_length_ = 0;
  for (size_t k = 0; k < terms_.size(); ++k) {
    terms_[k].history.assign(lengths[k] * terms_[k].dim, 0.0F);
    total += terms_[k].history.size();
    history_length_ = std::max(history_length_, lengths[k]);
  }
  if (total != obs_dim) {
    throw std::runtime_error(
            "The observation terms add up to " + std::to_string(total) + " values, but the '" +
            obs_input_ + "' input has " + std::to_string(obs_dim));
  }
  has_time_step_ = model.inputs.count("time_step") > 0;
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
  for (auto & t : terms_) {t.term->reset();}
}

bool MjlabPreprocessor::ready(const DataStore & state) const
{
  return std::all_of(
    terms_.begin(), terms_.end(), [&](const Term & t) {return t.term->ready(state);});
}

void MjlabPreprocessor::push(const DataStore & state, const TensorMap & last_outputs)
{
  // Oldest known stamp of the terms neither derived from previous outputs nor user commands.
  double obs_stamp = 0.0;
  for (size_t k = 0; k < terms_.size(); ++k) {
    auto & t = terms_[k];
    // Drop the oldest entry, append the current value as the newest.
    std::copy(t.history.data() + t.dim, t.history.data() + t.history.size(), t.history.data());
    float * newest = t.history.data() + t.history.size() - t.dim;
    double stamp = t.term->compute(state, last_outputs, newest);
    // A value derived from the previous outputs has the stamp of the previous observation.
    const bool derived = stamp < 0.0;  // kFromPreviousOutputs
    if (derived) {stamp = prev_obs_stamp_;}
    stamps_[k].stamp = stamp;
    stamps_[k].from_previous_outputs = derived;
    stamps_[k].receive_time = t.term->receive_time();
    if (!derived && !stamps_[k].user_command && stamp > 0.0 &&
      (obs_stamp <= 0.0 || stamp < obs_stamp))
    {
      obs_stamp = stamp;
    }
    if (!history_filled_) {
      // Like mjlab's CircularBuffer: the first push after clear back-fills the whole history.
      for (float * frame = t.history.data(); frame < newest; frame += t.dim) {
        std::copy(newest, newest + t.dim, frame);
      }
    }
  }
  history_filled_ = true;
  prev_obs_stamp_ = obs_stamp;
}

void MjlabPreprocessor::observe(const DataStore & state, const TensorMap & last_outputs)
{
  // Before the first compute() there is no history to keep current: it back-fills then.
  if (history_filled_ && ready(state)) {push(state, last_outputs);}
}

bool MjlabPreprocessor::compute(
  const DataStore & state, const TensorMap & last_outputs, TensorMap & inputs)
{
  if (!ready(state)) {return false;}
  push(state, last_outputs);
  auto & obs = inputs.at(obs_input_).data;
  size_t offset = 0;
  for (const auto & t : terms_) {
    std::copy(t.history.begin(), t.history.end(), obs.data() + offset);
    offset += t.history.size();
  }
  if (has_time_step_) {
    inputs.at("time_step").data[0] = static_cast<float>(step_);
  }
  ++step_;
  return true;
}

}  // namespace policy_framework_mjlab

PLUGINLIB_EXPORT_CLASS(policy_framework_mjlab::MjlabPreprocessor, policy_framework::Preprocessor)
