// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__CORE__STEP_REPLAYER_HPP_
#define POLICY_FRAMEWORK__CORE__STEP_REPLAYER_HPP_

#include <map>
#include <memory>
#include <string>

#include "policy_framework/msg/policy_step.hpp"
#include "policy_framework/plugin_interfaces.hpp"

namespace policy_framework
{

/**
 * @brief Runs the model inputs of recorded policy steps (msg::PolicyStep, see StepPublisher)
 *        through a model again and measures how far its outputs are from the recorded ones.
 *
 * E.g. to check that a model exported again, or converted to another runtime, answers what the
 * robot's model answered; or to see what a new version of a policy would have done on the
 * robot's real inputs. Not for the control cycle (it allocates).
 */
class StepReplayer
{
public:
  /// Difference between the replayed and the recorded values of one model output.
  struct OutputError
  {
    double max_error = 0.0;   ///< Largest absolute difference over every value and step.
    double mean_error = 0.0;  ///< Mean over the steps of the largest difference of the step.
    size_t steps = 0;         ///< Replayed steps that had this output.
    uint64_t worst_step = 0;  ///< PolicyStep::step of the step with max_error.
    std::string worst_policy;  ///< PolicyStep::policy of that step.
  };

  struct Summary
  {
    size_t replayed = 0;  ///< Steps run through the model.
    size_t skipped = 0;   ///< Steps whose model did not run, or of another policy.
    /// Per output of the model that the recorded steps also have.
    std::map<std::string, OutputError> outputs;
  };

  /**
   * @param inference A configured model.
   * @param policy Replay only the steps of this policy; empty: every step.
   */
  explicit StepReplayer(std::shared_ptr<Inference> inference, std::string policy = "");

  /**
   * @brief Run the recorded inputs of @p step through the model and compare the outputs.
   * @return false when the step is skipped: its status is not OK (the model did not run), or it
   *         belongs to another policy.
   * @throws std::runtime_error when its inputs do not match the model's (names, sizes), or the
   *         model fails.
   */
  bool replay(const msg::PolicyStep & step);

  const Summary & summary() const {return summary_;}
  /// @return Model outputs of the last replayed step.
  const TensorMap & outputs() const {return outputs_;}

private:
  std::shared_ptr<Inference> inference_;
  std::string policy_;
  TensorMap inputs_;
  TensorMap outputs_;
  Summary summary_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__CORE__STEP_REPLAYER_HPP_
