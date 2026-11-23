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

#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/plugins/stages/generic_stages.hpp"

namespace policy_framework_test
{

/**
 * @brief Test only: a ConcatPreprocessor that waits for data while the state entry `<name>.gate`
 *        (default `gate/value`) is missing or not greater than 0, e.g. to make a policy wait
 *        again after it ran.
 */
class GatedPreprocessor : public policy_framework::ConcatPreprocessor
{
public:
  void configure(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
    policy_framework::DataStore & state) override
  {
    ConcatPreprocessor::configure(ctx, model, state);
    gate_ = state.add(policy_framework::get_parameter<std::string>(ctx, "gate", "gate/value"));
  }

  bool compute(
    const policy_framework::DataStore & state, const policy_framework::TensorMap & last_outputs,
    policy_framework::TensorMap & inputs) override
  {
    if (!state.received(gate_) || state.get(gate_) <= 0.0) {return false;}
    return ConcatPreprocessor::compute(state, last_outputs, inputs);
  }

private:
  size_t gate_ = 0;
};

}  // namespace policy_framework_test

PLUGINLIB_EXPORT_CLASS(policy_framework_test::GatedPreprocessor, policy_framework::Preprocessor)
