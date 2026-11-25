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

#include "policy_framework/core/policy_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <fmt/ranges.h>

namespace policy_framework
{

void PolicyPipeline::add_policy(Policy policy)
{
  if (!policy.preprocessor || !policy.inference || !policy.postprocessor) {
    throw std::invalid_argument(fmt::format("Policy '{}' is missing a stage", policy.name));
  }
  for (const auto & e : entries_) {
    if (e.policy.name == policy.name) {
      throw std::invalid_argument(fmt::format("Duplicate policy name '{}'", policy.name));
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
    message = fmt::format(
      "Unknown policy '{}'. Available: {}", name, fmt::join(policy_names(), " "));
    return false;
  }
  const auto & active = entries_[active_.load()].policy;
  if (!active.interruptible) {
    message = fmt::format("Active policy '{}' is not interruptible", active.name);
    return false;
  }
  requested_.store(static_cast<int>(it - entries_.begin()));
  message = fmt::format("Switching to policy '{}'", name);
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
  for (auto & other : entries_) {
    if (&other != &e) {other.policy.preprocessor->observe(state, other.outputs);}
  }
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

  stage_times_ = StageTimes();
  using Clock = std::chrono::steady_clock;
  const auto us_since = [](Clock::time_point t) {
      return std::chrono::duration<double, std::micro>(Clock::now() - t).count();
    };
  auto start = Clock::now();
  const bool ready = e.policy.preprocessor->compute(state, e.outputs, e.inputs);
  stage_times_.preprocessor_us = us_since(start);
  if (!ready) {
    return Status::WAITING_FOR_DATA;
  }
  observation_stamp_ = 0.0;
  for (const auto & s : e.policy.preprocessor->input_stamps()) {
    if (!s.from_previous_outputs && !s.user_command && s.stamp > 0.0 &&
      (observation_stamp_ <= 0.0 || s.stamp < observation_stamp_))
    {
      observation_stamp_ = s.stamp;
    }
  }
  start = Clock::now();
  const bool ran = e.policy.inference->run(e.inputs, e.outputs);
  stage_times_.inference_us = us_since(start);
  if (!ran) {
    return Status::ERROR;
  }
  // A diverged model must not command the robot: NaN would go through every comparison below.
  for (const auto & [name, t] : e.outputs) {
    for (const float v : t.data) {
      if (!std::isfinite(v)) {return Status::ERROR;}
    }
  }
  start = Clock::now();
  const bool written = e.policy.postprocessor->compute(e.outputs, observation_stamp_, commands);
  stage_times_.postprocessor_us = us_since(start);
  if (!written) {
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
