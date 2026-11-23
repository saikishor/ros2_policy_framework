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

// Test-only observation term: the position of `imu_link` from PluginContext::kinematics.

#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/core/kinematics.hpp"
#include "policy_framework/observation_term.hpp"

namespace policy_framework_mjlab_test
{

class FramePositionTerm : public policy_framework::ObservationTerm
{
public:
  size_t configure(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo &,
    policy_framework::DataStore &) override
  {
    if (!ctx.kinematics) {throw std::runtime_error("FramePositionTerm needs kinematics");}
    kinematics_ = ctx.kinematics;
    frame_ = kinematics_->frame("imu_link");
    return 3;
  }

  bool ready(const policy_framework::DataStore &) override {return kinematics_->ready();}

  double compute(
    const policy_framework::DataStore &, const policy_framework::TensorMap &,
    float * out) override
  {
    const auto & p = kinematics_->transform(frame_).translation();
    for (int i = 0; i < 3; ++i) {out[i] = static_cast<float>(p[i]);}
    return kinematics_->stamp();
  }

private:
  std::shared_ptr<policy_framework::Kinematics> kinematics_;
  size_t frame_ = 0;
};

}  // namespace policy_framework_mjlab_test

PLUGINLIB_EXPORT_CLASS(
  policy_framework_mjlab_test::FramePositionTerm, policy_framework::ObservationTerm)
