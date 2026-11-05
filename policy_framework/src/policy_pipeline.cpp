// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/policy_pipeline.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace policy_framework
{

void PolicyPipeline::add_policy(Policy policy)
{
  if (!policy.preprocessor || !policy.inference || !policy.postprocessor) {
    throw std::invalid_argument("Policy '" + policy.name + "' is missing a stage");
  }
  for (const auto & e : entries_) {
    if (e.policy.name == policy.name) {
      throw std::invalid_argument("Duplicate policy name '" + policy.name + "'");
    }
  }
  Entry entry;
  entry.inputs = policy.inference->model().inputs;
  entry.outputs = policy.inference->model().outputs;
  entry.policy = std::move(policy);
  entries_.push_back(std::move(entry));
}

bool PolicyPipeline::request_switch(const std::string & name, std::string & message)
{
  const auto it = std::find_if(
    entries_.begin(), entries_.end(), [&](const Entry & e) {return e.policy.name == name;});
  if (it == entries_.end()) {
    message = "Unknown policy '" + name + "'. Available: ";
    for (const auto & n : policy_names()) {message += n + " ";}
    return false;
  }
  const auto & active = entries_[active_.load()].policy;
  if (!active.interruptible) {
    message = "Active policy '" + active.name + "' is not interruptible";
    return false;
  }
  requested_.store(static_cast<int>(it - entries_.begin()));
  message = "Switching to policy '" + name + "'";
  return true;
}

PolicyPipeline::Status PolicyPipeline::step(const DataStore & state, DataStore & commands)
{
  if (entries_.empty()) {
    return Status::ERROR;
  }
  if (restart_requested_.exchange(false)) {
    for (auto & entry : entries_) {entry.needs_clear = true;}
  }
  if (const int req = requested_.exchange(-1); req >= 0) {
    active_.store(static_cast<size_t>(req));
    needs_reset_ = true;
  }
  auto & e = entries_[active_.load()];
  if (e.needs_clear) {
    for (auto & [name, t] : e.outputs) {std::fill(t.data.begin(), t.data.end(), 0.0F);}
    e.policy.preprocessor->clear();
    e.needs_clear = false;
  }
  if (needs_reset_) {
    e.policy.preprocessor->reset();
    e.policy.postprocessor->reset();
    steps_ = 0;
    needs_reset_ = false;
  }

  if (!e.policy.preprocessor->compute(state, e.outputs, e.inputs)) {
    return Status::WAITING_FOR_DATA;
  }
  observation_stamp_ = 0.0;
  for (const auto & s : e.policy.preprocessor->input_stamps()) {
    if (!s.from_previous_outputs && s.stamp > 0.0 &&
      (observation_stamp_ <= 0.0 || s.stamp < observation_stamp_))
    {
      observation_stamp_ = s.stamp;
    }
  }
  if (!e.policy.inference->run(e.inputs, e.outputs)) {
    return Status::ERROR;
  }
  if (!e.policy.postprocessor->compute(e.outputs, observation_stamp_, commands)) {
    return Status::ERROR;
  }

  ++steps_;
  if (e.policy.max_steps > 0 && steps_ >= e.policy.max_steps) {
    // Bypasses request_switch() on purpose: the interruptible check must not block the return.
    requested_.store(0);
  }
  return Status::OK;
}

std::vector<std::string> PolicyPipeline::policy_names() const
{
  std::vector<std::string> names;
  for (const auto & e : entries_) {names.push_back(e.policy.name);}
  return names;
}

}  // namespace policy_framework
