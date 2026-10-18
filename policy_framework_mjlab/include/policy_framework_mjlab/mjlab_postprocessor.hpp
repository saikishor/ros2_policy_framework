// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK_MJLAB__MJLAB_POSTPROCESSOR_HPP_
#define POLICY_FRAMEWORK_MJLAB__MJLAB_POSTPROCESSOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "policy_framework/plugin_interfaces.hpp"
#include "rclcpp/rclcpp.hpp"

namespace policy_framework_mjlab
{

/**
 * @brief pid_controller gain parameters `gains.<joint>.p` / `gains.<joint>.d`.
 * @throws std::invalid_argument when the sizes differ or there is no gain.
 */
/// Gains of one policy for its gains controller.
struct GainsRequest
{
  std::string policy;
  std::vector<rclcpp::Parameter> gains;
};

/// Sends gains to one controller, latest request wins (shared by the policies of a node).
class GainsSender;

std::vector<rclcpp::Parameter> make_gain_parameters(
  const std::vector<std::string> & joints, const std::vector<double> & stiffness,
  const std::vector<double> & damping);

/**
 * @brief Converts mjlab actions into joint position references.
 *
 * The actuated joints are the metadata `joint_names` minus the `excluded_joints` parameter
 * (e.g. passive joints of a closed kinematic chain); there must be exactly one per action.
 * For each of them: `<joint>/<position_interface>` = action_scale * action + default_joint_pos,
 * and, unless `velocity_interface` is empty, `<joint>/<velocity_interface>` = 0.
 *
 * Gains: when `gains_controller` is set (e.g. `/rl_impedance_controller`), the metadata
 * `joint_stiffness` / `joint_damping` of the policy are sent as that pid_controller's
 * `gains.<joint>.p/.d` parameters every time the policy is activated, from a non real-time
 * timer that retries every 0.5 s until the controller's parameter service is available. When
 * policies are switched faster than that, only the gains of the latest activated policy are
 * sent.
 *
 * With `gains_as_commands` the same gains are also written as command entries
 * `<joint>/stiffness` and `<joint>/damping` on every step, for a PD law in the controller
 * (policy_framework/PdEffortExecutor): gains then follow policy switches with no parameter
 * service involved. Missing metadata gains are then a configuration error.
 *
 * Parameters (under `<policy>.`): `excluded_joints` (string array, default empty),
 * `position_interface` (default `position`), `velocity_interface` (default `velocity`),
 * `gains_controller` (default empty: gains are not sent), `gains_as_commands` (default
 * false).
 */
class MjlabPostprocessor : public policy_framework::Postprocessor
{
public:
  ~MjlabPostprocessor() override;

  /// @throws std::runtime_error on missing metadata or size mismatch.
  void configure(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
    policy_framework::DataStore & commands) override;
  /// Real-time safe: only flags the gains to be sent.
  void reset() override;
  /// Writes the commands with @p stamp (the observation stamp).
  bool compute(
    const policy_framework::TensorMap & outputs, double stamp,
    policy_framework::DataStore & commands) override;

private:
  std::vector<double> scale_, offset_;
  std::vector<size_t> position_idx_;
  std::vector<size_t> velocity_idx_;
  std::vector<size_t> gain_idx_;       ///< stiffness, damping entries per actuated joint.
  std::vector<double> gain_values_;

  rclcpp::Logger logger_{rclcpp::get_logger("MjlabPostprocessor")};
  GainsRequest gains_request_;
  std::shared_ptr<GainsSender> gains_sender_;
};

}  // namespace policy_framework_mjlab

#endif  // POLICY_FRAMEWORK_MJLAB__MJLAB_POSTPROCESSOR_HPP_
