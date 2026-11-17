// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#include "policy_framework/core/step_replayer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace policy_framework
{

StepReplayer::StepReplayer(std::shared_ptr<Inference> inference, std::string policy)
: inference_(std::move(inference)), policy_(std::move(policy)),
  inputs_(inference_->model().inputs), outputs_(inference_->model().outputs)
{
}

bool StepReplayer::replay(const msg::PolicyStep & step)
{
  if (step.status != msg::PolicyStep::STATUS_OK || (!policy_.empty() && step.policy != policy_)) {
    ++summary_.skipped;
    return false;
  }
  for (auto & [name, tensor] : inputs_) {
    const auto it = std::find_if(
      step.inputs.begin(), step.inputs.end(), [&](const auto & t) {return t.name == name;});
    if (it == step.inputs.end()) {
      throw std::runtime_error("Step " + std::to_string(step.step) + " has no input '" + name + "'");
    }
    if (it->data.size() != tensor.data.size()) {
      throw std::runtime_error(
              "Input '" + name + "' of step " + std::to_string(step.step) + " has " +
              std::to_string(it->data.size()) + " values, the model " +
              std::to_string(tensor.data.size()));
    }
    std::copy(it->data.begin(), it->data.end(), tensor.data.begin());
  }
  if (!inference_->run(inputs_, outputs_)) {
    throw std::runtime_error("The model failed on step " + std::to_string(step.step));
  }
  ++summary_.replayed;
  for (const auto & recorded : step.outputs) {
    const auto it = outputs_.find(recorded.name);
    if (it == outputs_.end()) {continue;}
    const auto & data = it->second.data;
    if (recorded.data.size() != data.size()) {
      throw std::runtime_error(
              "Output '" + recorded.name + "' of step " + std::to_string(step.step) + " has " +
              std::to_string(recorded.data.size()) + " values, the model " +
              std::to_string(data.size()));
    }
    double step_error = 0.0;
    for (size_t i = 0; i < data.size(); ++i) {
      step_error = std::max(
        step_error, std::abs(static_cast<double>(data[i]) - static_cast<double>(recorded.data[i])));
    }
    auto & error = summary_.outputs[recorded.name];
    ++error.steps;
    if (error.steps == 1 || step_error > error.max_error) {
      error.max_error = step_error;
      error.worst_step = step.step;
      error.worst_policy = step.policy;
    }
    error.mean_error += (step_error - error.mean_error) / static_cast<double>(error.steps);
  }
  return true;
}

}  // namespace policy_framework
