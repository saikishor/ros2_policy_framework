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

#include "policy_framework/hosts/policy_runner.hpp"

#include <algorithm>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <fmt/format.h>

#include "policy_framework/kinematics_parameters.hpp"
#include "policy_framework/policy_runner_parameters.hpp"

namespace policy_framework
{
namespace
{

template<typename T>
std::unique_ptr<pluginlib::ClassLoader<T>> make_loader(const std::string & base)
{
  return std::make_unique<pluginlib::ClassLoader<T>>("policy_framework", "policy_framework::" + base);
}

}  // namespace

void PolicyRunner::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, bool load_executors,
  const std::string & robot_description, bool step_in_other_thread)
{
  logger_ = node->get_logger();
  const auto & logger = logger_;
  // Reconfiguration starts from scratch: drop plugins before their loaders.
  source_ages_.clear();
  input_age_reactions_.clear();
  policy_inputs_.clear();
  policy_infos_.clear();
  sources_.clear();
  executors_.clear();
  checks_.clear();
  kinematics_ = KinematicsFeed();
  policy_kinematics_ = KinematicsFeed();
  pipeline_ = std::make_unique<PolicyPipeline>();
  state_ = DataStore();
  commands_ = DataStore();
  source_loader_ = make_loader<InputSource>("InputSource");
  preprocessor_loader_ = make_loader<Preprocessor>("Preprocessor");
  inference_loader_ = make_loader<Inference>("Inference");
  postprocessor_loader_ = make_loader<Postprocessor>("Postprocessor");
  executor_loader_ = make_loader<Executor>("Executor");
  safety_check_loader_ = make_loader<SafetyCheck>("SafetyCheck");

  // Declares and validates sources, policies, executors and their per-name parameters.
  const auto params = policy_runner::ParamListener(node).get_params();
  executor_types_.clear();
  for (const auto & name : params.executors) {
    executor_types_.emplace_back(name, params.executors_map.at(name).type);
  }

  const auto & safety = params.safety_entry;
  if (!safety.empty()) {state_.add(safety);}
  // Before the plugins, so that their CommandGate finds it.
  neutral_idx_ = state_.add(kNeutralCommandsEntry);
  command_gate_ = CommandGate(PluginContext{node, "", nullptr, safety});
  commands_accepted_ = true;

  // Before the policies: their plugins look frames up while configuring.
  configure_kinematics(node, robot_description, step_in_other_thread);

  for (const auto & name : params.sources) {
    const auto & type = params.sources_map.at(name).type;
    auto source = source_loader_->createSharedInstance(type);
    source->configure({node, name, nullptr, safety});
    sources_.push_back(source);
    RCLCPP_INFO(logger, "Input source '%s' (%s)", name.c_str(), type.c_str());
  }

  for (const auto & name : params.policies) {
    const PluginContext ctx{node, name, policy_kinematics_.model, safety};
    const auto & stages = params.policies_map.at(name);
    Policy p;
    p.name = name;
    p.inference = inference_loader_->createSharedInstance(stages.inference);
    p.inference->configure(ctx);
    p.preprocessor = preprocessor_loader_->createSharedInstance(stages.preprocessor);
    p.preprocessor->configure(ctx, p.inference->model(), state_);
    PolicyInputs inputs{name, {}};
    for (const auto & s : p.preprocessor->input_stamps()) {
      inputs.inputs.push_back(s.name);
      if (s.max_age > 0.0) {input_age_reactions_.push_back(s.max_age_on_fault);}
    }
    policy_inputs_.push_back(std::move(inputs));
    p.postprocessor = postprocessor_loader_->createSharedInstance(stages.postprocessor);
    p.postprocessor->configure(ctx, p.inference->model(), commands_);
    p.max_steps = static_cast<int>(stages.max_steps);
    p.interruptible = stages.interruptible;
    policy_framework_msgs::msg::PolicyInfo info;
    info.name = name;
    info.preprocessor = stages.preprocessor;
    info.inference = stages.inference;
    info.postprocessor = stages.postprocessor;
    info.max_steps = p.max_steps;
    info.interruptible = p.interruptible;
    policy_infos_.push_back(std::move(info));
    RCLCPP_INFO(
      logger, "Policy '%s': max_steps=%d interruptible=%s", name.c_str(), p.max_steps,
      p.interruptible ? "true" : "false");
    pipeline_->add_policy(std::move(p));
  }

  if (load_executors) {
    for (const auto & [name, type] : executor_types_) {
      auto executor = executor_loader_->createSharedInstance(type);
      executor->configure({node, name}, commands_);
      executors_.push_back(executor);
      RCLCPP_INFO(logger, "Executor '%s' (%s)", name.c_str(), type.c_str());
    }
  }

  configure_safety(node, params, robot_description);

  switch_service_ = node->create_service<policy_framework_msgs::srv::SwitchPolicy>(
    "~/switch_policy",
    [this](
      const std::shared_ptr<policy_framework_msgs::srv::SwitchPolicy::Request> req,
      std::shared_ptr<policy_framework_msgs::srv::SwitchPolicy::Response> res) {
      res->success = pipeline_->request_switch(req->policy_name, res->message);
      RCLCPP_INFO(logger_, "switch_policy: %s", res->message.c_str());
    });
  list_service_ = node->create_service<policy_framework_msgs::srv::ListPolicies>(
    "~/list_policies",
    [this](
      const std::shared_ptr<policy_framework_msgs::srv::ListPolicies::Request>,
      std::shared_ptr<policy_framework_msgs::srv::ListPolicies::Response> res) {
      res->policies = policy_infos_;
    });
  reset_service_ = node->create_service<std_srvs::srv::Trigger>(
    "~/reset_safety",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      // Only a request: the latch and the checks belong to the control thread.
      reset_requested_.store(true);
      res->success = true;
      res->message =
        "Safety reset requested: applied at the next cycle, whatever still fails latches again";
      RCLCPP_INFO(logger_, "reset_safety: %s", res->message.c_str());
    });
  // Latched, so that a late subscriber gets the current policy (published only on changes).
  active_pub_ = std::make_unique<
    realtime_tools::RealtimePublisher<policy_framework_msgs::msg::ActivePolicies>>(
    node->create_publisher<policy_framework_msgs::msg::ActivePolicies>(
      "~/active_policies", rclcpp::QoS(1).transient_local()));
  active_msg_.policies.assign(1, "");
  active_pending_ = false;

  latched_.reset();
  reported_.reset();
  last_status_ = PolicyPipeline::Status::OK;
  last_policy_.clear();
  RCLCPP_INFO(
    logger, "%zu state entries, %zu command entries, active policy '%s'", state_.size(),
    commands_.size(), pipeline_->active_policy().c_str());
}

void PolicyRunner::configure_safety(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const policy_runner::Params & params,
  const std::string & robot_description)
{
  // The parameter validation allows only the reaction names: the lookups cannot fail.
  const auto reaction = [](const std::string & name) {return *fault_reaction_from_string(name);};
  policy_failure_reaction_ = reaction(params.on_policy_failure);
  const auto & s = params.safety;
  tilt_on_fault_ = reaction(s.tilt_on_fault);
  position_on_fault_ = reaction(s.joint_position_on_fault);
  input_age_on_fault_ = reaction(s.input_age_on_fault);

  // After the policies: the monitor checks the joints they command, and the plugins may look
  // for command entries.
  SafetyMonitor::Config config;
  config.max_tilt = s.max_tilt;
  config.imu = s.imu;
  config.position_margin = s.position_margin;
  config.max_input_age = s.max_input_age;
  if (s.joint_position_limits) {
    if (robot_description.empty()) {
      throw std::runtime_error("safety.joint_position_limits needs a robot description");
    }
    config.joint_limits = SafetyMonitor::joint_limits(robot_description);
  }
  monitor_.configure(config, commands_, state_);
  monitor_reactions_.clear();
  if (s.max_tilt > 0.0) {monitor_reactions_.push_back(tilt_on_fault_);}
  if (s.joint_position_limits) {monitor_reactions_.push_back(position_on_fault_);}
  if (s.max_input_age > 0.0) {monitor_reactions_.push_back(input_age_on_fault_);}
  monitor_enabled_ = !monitor_reactions_.empty();

  for (size_t i = 0; i < params.sources.size(); ++i) {
    const auto & entry = params.sources_map.at(params.sources[i]);
    if (entry.max_age > 0.0) {
      source_ages_.push_back(
        {params.sources[i], sources_[i], entry.max_age, reaction(entry.max_age_on_fault)});
    }
  }

  // The monitor checks a policy may disable, by the name of their parameter.
  const std::vector<std::pair<std::string, SafetyMonitor::Check>> monitor_checks = {
    {"tilt", SafetyMonitor::Check::TILT},
    {"joint_position_limits", SafetyMonitor::Check::JOINT_POSITION},
    {"max_input_age", SafetyMonitor::Check::INPUT_AGE}};
  for (const auto & name : params.safety_checks) {
    for (const auto & m : monitor_checks) {
      if (name == m.first) {
        throw std::runtime_error(
                fmt::format(
                  "safety_checks: '{}' is the name of a monitor check, which "
                  "disabled_safety_checks would not tell apart", name));
      }
    }
    const auto & entry = params.safety_checks_map.at(name);
    Check c;
    c.name = name;
    c.check = safety_check_loader_->createSharedInstance(entry.type);
    c.check->configure({node, name, kinematics_.model, params.safety_entry}, commands_, state_);
    c.on_fault = reaction(entry.on_fault);
    c.declared = c.check->possible_reactions();
    RCLCPP_INFO(
      logger_, "Safety check '%s' (%s), on_fault: %s", name.c_str(), entry.type.c_str(),
      to_string(c.on_fault));
    checks_.push_back(std::move(c));
  }

  policy_safety_.clear();
  for (const auto & policy : params.policies) {
    PolicySafety p;
    p.checks.assign(checks_.size(), false);
    for (const auto & name : params.policies_map.at(policy).disabled_safety_checks) {
      bool found = false;
      for (const auto & m : monitor_checks) {
        if (name == m.first) {
          p.monitor.set(SafetyMonitor::bit(m.second));
          found = true;
        }
      }
      for (size_t i = 0; i < checks_.size(); ++i) {
        if (name == checks_[i].name) {
          p.checks[i] = true;
          found = true;
        }
      }
      if (!found) {
        throw std::runtime_error(
                fmt::format(
                  "{}.disabled_safety_checks: unknown check '{}' (a safety_checks name, tilt, "
                  "joint_position_limits or max_input_age)", policy, name));
      }
      RCLCPP_INFO(logger_, "Policy '%s' disables safety check '%s'", policy.c_str(), name.c_str());
    }
    policy_safety_.push_back(std::move(p));
  }
  // Reserved now, so that a failing check does not allocate for its reason in most cases.
  reason_.reserve(256);
  check_reason_.reserve(256);

  fault_slots_.clear();
  for (const auto & m : monitor_checks) {fault_slots_.push_back({m.first, std::nullopt, ""});}
  source_slots_ = fault_slots_.size();
  for (const auto & a : source_ages_) {fault_slots_.push_back({a.name, std::nullopt, ""});}
  check_slots_ = fault_slots_.size();
  for (const auto & c : checks_) {fault_slots_.push_back({c.name, std::nullopt, ""});}
  input_slot_ = fault_slots_.size();
  fault_slots_.push_back({"policy_input", std::nullopt, ""});
  failure_slot_ = fault_slots_.size();
  fault_slots_.push_back({"policy_failure", std::nullopt, ""});
  for (auto & f : fault_slots_) {f.reason.reserve(256);}
  published_slots_.assign(fault_slots_.size(), std::nullopt);
  published_reaction_.reset();

  clock_ = node->get_clock();
  // Latched, so that a user interface started during a fault shows it.
  status_pub_ = std::make_unique<
    realtime_tools::RealtimePublisher<policy_framework_msgs::msg::SafetyStatusArray>>(
    node->create_publisher<policy_framework_msgs::msg::SafetyStatusArray>(
      "~/safety_status", rclcpp::QoS(1).transient_local()));
  // The initial state, every check passing.
  status_msg_ = policy_framework_msgs::msg::SafetyStatusArray();
  status_msg_.stamp = clock_->now();
  status_msg_.active_policy = pipeline_->active_policy();
  status_pending_ = !status_pub_->try_publish(status_msg_);
}

void PolicyRunner::restart()
{
  pipeline_->restart();
  pipeline_->write_initial_commands(commands_);
  clear_safety();
  reset_requested_.store(false);
}

void PolicyRunner::clear_safety()
{
  set_neutral_commands(false, 0.0);
  latched_.reset();
  reported_.reset();
  for (auto & c : checks_) {c.check->reset();}
  // Published by the next update_safety().
  for (auto & f : fault_slots_) {f.reaction.reset();}
}

bool PolicyRunner::apply_safety_reset()
{
  if (!reset_requested_.exchange(false)) {return false;}
  // Only stops are latched; neutral commands are re-evaluated on every cycle.
  const bool stopped = latched_.has_value();
  // Not write_initial_commands() (outside the control cycle only): the executors hold with the
  // commands the store already has until the first step.
  if (stopped) {pipeline_->restart();}
  clear_safety();
  // ponytail: logs from the control cycle, on an operator's request only, like report().
  RCLCPP_INFO(logger_, "Safety reset%s", stopped ? ": restarting the default policy" : "");
  return stopped;
}

FaultReaction PolicyRunner::reaction_of(
  const SafetyCheck::Verdict & verdict, FaultReaction configured,
  const std::vector<FaultReaction> & declared)
{
  if (!verdict.reaction) {return configured;}
  // An undeclared reaction was not checked against the executors while configuring.
  return std::find(declared.begin(), declared.end(), *verdict.reaction) != declared.end() ?
         *verdict.reaction : FaultReaction::ERROR;
}

std::vector<FaultReaction> PolicyRunner::fault_reactions() const
{
  std::vector<FaultReaction> all{policy_failure_reaction_};
  all.insert(all.end(), monitor_reactions_.begin(), monitor_reactions_.end());
  for (const auto & s : source_ages_) {all.push_back(s.on_fault);}
  all.insert(all.end(), input_age_reactions_.begin(), input_age_reactions_.end());
  for (const auto & c : checks_) {
    all.push_back(c.on_fault);
    all.insert(all.end(), c.declared.begin(), c.declared.end());
    // An undeclared reaction of a verdict becomes ERROR.
    all.push_back(FaultReaction::ERROR);
  }
  std::sort(all.begin(), all.end());
  all.erase(std::unique(all.begin(), all.end()), all.end());
  return all;
}

std::optional<FaultReaction> PolicyRunner::update_safety(double observation_stamp, double now)
{
  std::optional<FaultReaction> worst;
  // The policy whose commands are applied: a requested switch takes effect at the start of the
  // next step, after this, so a new policy's exemptions apply from the cycle after its first
  // step, and the checks it disabled come back from the cycle after the next policy's first step.
  const auto & disabled = policy_safety_[pipeline_->active_index()];
  const auto v = monitor_.check(state_, observation_stamp, now, disabled.monitor);
  if (v) {
    worst = v->check == SafetyMonitor::Check::TILT ? tilt_on_fault_ :
      v->check == SafetyMonitor::Check::JOINT_POSITION ? position_on_fault_ :
      input_age_on_fault_;
    reason_ = v->describe();
  }
  for (size_t i = 0; i < source_slots_; ++i) {
    // The slots are in the order of SafetyMonitor::Check.
    if (set_fault(i, v && SafetyMonitor::bit(v->check) == i ? worst : std::nullopt)) {
      fault_slots_[i].reason = reason_;
    }
  }
  for (size_t i = 0; i < source_ages_.size(); ++i) {
    const auto & s = source_ages_[i];
    const double received = s.source->receive_time();
    // Nothing received yet: the policy waits for it, this is not a fault.
    const bool stale = received > 0.0 && now - received > s.max_age;
    const bool new_fault = set_fault(source_slots_ + i, stale ? std::optional{s.on_fault} :
        std::nullopt);
    if (!stale) {continue;}
    const bool most_severe = !worst || s.on_fault > *worst;
    if (!most_severe && !new_fault) {continue;}
    const auto reason = fmt::format(
      "source '{}': no message for {} s, above its max_age", s.name, now - received);
    if (new_fault) {fault_slots_[source_slots_ + i].reason = reason;}
    if (most_severe) {
      worst = s.on_fault;
      reason_ = reason;
    }
  }
  // Every check runs, also after a failing one: the most severe reaction wins.
  for (size_t i = 0; i < checks_.size(); ++i) {
    auto & c = checks_[i];
    if (disabled.checks[i]) {
      // Not run at all: a check that fails throughout a jump would allocate its reason on every
      // cycle.
      c.skipped = true;
      set_fault(check_slots_ + i, std::nullopt);
      continue;
    }
    if (c.skipped) {
      // Its filters and previous values are from before the policy that disabled it.
      c.check->reset();
      c.skipped = false;
    }
    const auto verdict = c.check->check(state_, now, check_reason_);
    if (verdict.safe) {
      set_fault(check_slots_ + i, std::nullopt);
      continue;
    }
    const auto r = reaction_of(verdict, c.on_fault, c.declared);
    if (set_fault(check_slots_ + i, r)) {
      fault_slots_[check_slots_ + i].reason.assign(c.name).append(": ").append(check_reason_);
    }
    if (!worst || r > *worst) {
      worst = r;
      reason_ = fmt::format("{}: {}", c.name, check_reason_);
    }
  }
  if (worst && *worst != FaultReaction::NEUTRAL_COMMANDS) {
    escalate(*worst);
  }
  set_neutral_commands(!latched_ && worst == FaultReaction::NEUTRAL_COMMANDS, now);
  const auto effective = latched_ ? latched_ : worst;
  report(effective, reason_);
  publish_status();
  return effective;
}

std::optional<size_t> PolicyRunner::stale_input(const InputStamps & stamps, double now)
{
  std::optional<size_t> worst;
  for (size_t i = 0; i < stamps.size(); ++i) {
    const auto & s = stamps[i];
    // Nothing received yet: the policy waits for it, this is not a fault.
    if (s.max_age <= 0.0 || s.receive_time <= 0.0 || now - s.receive_time <= s.max_age) {
      continue;
    }
    if (!worst || s.max_age_on_fault > stamps[*worst].max_age_on_fault) {worst = i;}
  }
  return worst;
}

std::optional<FaultReaction> PolicyRunner::check_input_ages(double now)
{
  const auto & stamps = pipeline_->input_stamps();
  if (const auto i = stale_input(stamps, now)) {
    const auto & s = stamps[*i];
    escalate(s.max_age_on_fault);
    const auto reason = fmt::format(
      "input '{}' of policy '{}': no message for {} s, above its max_age", s.name,
      pipeline_->active_policy(), now - s.receive_time);
    if (set_fault(input_slot_, s.max_age_on_fault)) {fault_slots_[input_slot_].reason = reason;}
    report(latched_, reason);
  }
  publish_status();
  return latched_;
}

FaultReaction PolicyRunner::policy_failure(const std::string & reason)
{
  escalate(policy_failure_reaction_);
  if (set_fault(failure_slot_, policy_failure_reaction_)) {
    fault_slots_[failure_slot_].reason = reason;
  }
  report(latched_, reason);
  publish_status();
  return *latched_;
}

void PolicyRunner::escalate(FaultReaction reaction)
{
  if (!latched_ || reaction > *latched_) {latched_ = reaction;}
}

void PolicyRunner::report(std::optional<FaultReaction> effective, const std::string & reason)
{
  // ponytail: logs from the control cycle on changes only (rare), like step(); not real-time
  // safe.
  if (effective == reported_) {return;}
  if (effective) {
    RCLCPP_ERROR(logger_, "Safety: %s, reaction: %s", reason.c_str(), to_string(*effective));
  } else {
    RCLCPP_INFO(logger_, "Safety: every check passes again");
  }
  reported_ = effective;
}

bool PolicyRunner::set_fault(size_t slot, std::optional<FaultReaction> reaction)
{
  auto & f = fault_slots_[slot];
  const bool stop = f.reaction && *f.reaction != FaultReaction::NEUTRAL_COMMANDS;
  if (stop && (!reaction || *reaction <= *f.reaction)) {return false;}
  if (reaction == f.reaction) {return false;}
  f.reaction = reaction;
  return reaction.has_value();
}

void PolicyRunner::publish_status()
{
  bool changed = reported_ != published_reaction_;
  for (size_t i = 0; i < fault_slots_.size() && !changed; ++i) {
    changed = fault_slots_[i].reaction != published_slots_[i];
  }
  if (changed) {
    // ponytail: builds the message on changes only (rare), allocating like report(); not
    // real-time safe.
    published_reaction_ = reported_;
    status_msg_.stamp = clock_->now();
    status_msg_.active_policy = pipeline_->active_policy();
    status_msg_.reaction = reported_ ? to_string(*reported_) : "";
    status_msg_.statuses.clear();
    for (size_t i = 0; i < fault_slots_.size(); ++i) {
      const auto & f = fault_slots_[i];
      published_slots_[i] = f.reaction;
      if (!f.reaction) {continue;}
      policy_framework_msgs::msg::SafetyStatus s;
      s.name = f.name;
      s.reaction = to_string(*f.reaction);
      s.reason = f.reason;
      status_msg_.statuses.push_back(std::move(s));
    }
    status_pending_ = true;
  }
  // Retried on the next cycles while the previous message is still being published: the topic
  // is latched, a lost change would stay wrong until the next one.
  // ponytail: never waits (the controller is deactivated from the real-time loop), so a second
  // change in the last cycle before an `error` deactivation is not sent until the next change
  // (e.g. a reactivation); a non-real-time retry timer if that case ever matters.
  if (status_pending_ && status_pub_->try_publish(status_msg_)) {status_pending_ = false;}
}

std::vector<std::string> PolicyRunner::unprovided_state() const
{
  std::vector<std::string> missing;
  for (const auto & name : state_.names()) {
    if (name == kNeutralCommandsEntry) {continue;}  // written by the runner itself
    if (std::none_of(
        sources_.begin(), sources_.end(), [&](const auto & s) {return s->provides(name);}))
    {
      missing.push_back(name);
    }
  }
  return missing;
}

void PolicyRunner::configure_kinematics(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & robot_description,
  bool step_in_other_thread)
{
  kinematics_position_idx_.clear();
  kinematics_velocity_idx_.clear();
  const auto params = kinematics::ParamListener(node, "kinematics").get_params();
  if (!params.enabled) {return;}
  if (robot_description.empty()) {
    throw std::runtime_error("kinematics.enabled needs a robot description");
  }
  auto make = [&]() {
      KinematicsFeed feed;
      feed.model = std::make_shared<Kinematics>(robot_description, params.joints);
      const auto n = static_cast<Eigen::Index>(feed.model->joint_names().size());
      feed.q = Eigen::VectorXd::Zero(n);
      feed.v = Eigen::VectorXd::Zero(n);
      return feed;
    };
  kinematics_ = make();
  // A step in another thread reads a copy of the state: its plugins need the kinematics of
  // that copy, not of whatever the host's thread has moved on to meanwhile.
  policy_kinematics_ = step_in_other_thread ? make() : kinematics_;
  for (const auto & j : kinematics_.model->joint_names()) {
    kinematics_position_idx_.push_back(state_.add(j + "/position"));
    kinematics_velocity_idx_.push_back(state_.add(j + "/velocity"));
  }
  RCLCPP_INFO(logger_, "Kinematics over %zu joints", kinematics_.model->joint_names().size());
}

void PolicyRunner::read_sources(double now)
{
  for (const auto & s : sources_) {s->read(state_, now);}
}

void PolicyRunner::update_kinematics(KinematicsFeed & feed, const DataStore & state) const
{
  if (!feed.model || !state.all_received(kinematics_position_idx_) ||
    !state.all_received(kinematics_velocity_idx_))
  {
    return;
  }
  double stamp = 0.0;
  for (size_t j = 0; j < kinematics_position_idx_.size(); ++j) {
    const auto p = kinematics_position_idx_[j];
    const auto v = kinematics_velocity_idx_[j];
    feed.q[static_cast<Eigen::Index>(j)] = state.get(p);
    feed.v[static_cast<Eigen::Index>(j)] = state.get(v);
    for (const auto i : {p, v}) {
      if (stamp <= 0.0 || state.stamp(i) < stamp) {stamp = state.stamp(i);}
    }
  }
  // Skipped (false) while a reader in another thread holds the lock: the model keeps the
  // previous state until the next update. Expected, so not logged (this runs at the control
  // rate).
  feed.model->update(feed.q, feed.v, stamp);
}

void PolicyRunner::update_kinematics() {update_kinematics(kinematics_, state_);}

PolicyPipeline::Status PolicyRunner::step(const DataStore & state, DataStore & commands)
{
  if (const bool accepted = command_gate_.open(state); accepted != commands_accepted_) {
    // Logged on transitions only, like the ones below.
    RCLCPP_WARN_EXPRESSION(
      logger_, !accepted, "Ignoring user commands (safety entry 0 or neutral commands)");
    RCLCPP_INFO_EXPRESSION(logger_, accepted, "Accepting user commands");
    commands_accepted_ = accepted;
  }
  if (policy_kinematics_.model != kinematics_.model) {update_kinematics(policy_kinematics_, state);}
  PolicyPipeline::Status status;
  try {
    status = pipeline_->step(state, commands);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Policy step failed: %s", e.what());
    status = PolicyPipeline::Status::ERROR;
  }
  // ponytail: logs from the control cycle on transitions only (rare), not real-time safe; move
  // to a realtime publisher if a robot ever shows jitter on switches.
  const auto & active = pipeline_->active_policy();
  if (active != last_policy_) {
    RCLCPP_INFO(logger_, "Active policy: '%s'", active.c_str());
    last_policy_ = active;
    // ponytail: the name copy may allocate, on transitions only like the log above.
    active_msg_.policies[0] = active;
    active_pending_ = true;
  }
  // Retried on the next steps while the previous message is still being published: the topic
  // is latched, a lost change would stay wrong until the next switch.
  if (active_pending_ && active_pub_->try_publish(active_msg_)) {active_pending_ = false;}
  if (status != last_status_) {
    if (status == PolicyPipeline::Status::WAITING_FOR_DATA) {
      RCLCPP_WARN(logger_, "Waiting for input data of policy '%s'", active.c_str());
    } else if (status == PolicyPipeline::Status::OK) {
      RCLCPP_INFO(logger_, "Policy '%s' is running", active.c_str());
    }
    last_status_ = status;
  }
  return status;
}

void PolicyRunner::write_executors()
{
  for (const auto & e : executors_) {e->write(commands_);}
}

}  // namespace policy_framework
