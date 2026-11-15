// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK_MJLAB__MJLAB_PREPROCESSOR_HPP_
#define POLICY_FRAMEWORK_MJLAB__MJLAB_PREPROCESSOR_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "policy_framework/observation_term.hpp"
#include "policy_framework/plugin_interfaces.hpp"

namespace policy_framework_mjlab
{

/**
 * @brief Gravity (0, 0, -1) of the world frame expressed in the body frame of orientation
 *        (w, x, y, z) (normalized here), i.e. mjlab's `projected_gravity`.
 */
void projected_gravity(double w, double x, double y, double z, float out[3]);

/**
 * @brief Builds the observation of an mjlab policy from the ONNX metadata
 *        (`observation_names`, `joint_names`, `default_joint_pos`, `command_names`).
 *
 * Observation terms (in `observation_names` order), read from these state entries:
 * - `base_ang_vel`: `<imu_name>/angular_velocity.{x,y,z}`
 * - `base_lin_acc`: `<imu_name>/linear_acceleration.{x,y,z}`
 * - `imu_projected_gravity` (or `projected_gravity`): from `<imu_name>/orientation.{w,x,y,z}`
 * - `base_lin_vel`: `<odometry_name>/linear.{x,y,z}`
 * - `joint_pos`: `<joint>/position` - default_joint_pos, for every metadata joint
 * - `joint_vel`: `<joint>/velocity`
 * - `actions`: the raw (unscaled) `actions` output of the previous step (zero after reset)
 * - `command`: with `command_names` = `motion`, the `joint_pos`,`joint_vel` outputs of the
 *   previous step (the robot joint positions/velocities on the first step after reset);
 *   otherwise a twist `<velocity_command_name>/linear.x`, `linear.y`, `angular.z`.
 *
 * History: the history length is the input size divided by the single observation size. Each
 * term is laid out as its own block, oldest first and newest last (matching mjlab's
 * per-term history with flattening). As mjlab's CircularBuffer does, the first step after
 * clear() (first activation, host restart) back-fills the whole history with the current
 * value. The history is kept when the policy is switched away and back (reset() only) and,
 * like pal_policy_deployer, keeps following the robot while another policy is active
 * (observe()), so the policy resumes with the real recent past. Back-filling a single frame
 * of a dynamic state instead (e.g. a jump landing) made history-3 policies fall in simulation.
 *
 * Term plugins: with `<policy>.terms.<term>.type` set, the term is a
 * policy_framework::ObservationTerm plugin of that class (parameters under
 * `<policy>.terms.<term>.`) instead of the built-in one. This replaces a built-in term (e.g.
 * `command` from a topic with policy_framework_mjlab/TwistCommandTerm) or adds a term the list
 * above does not know. History and layout apply to plugin terms as to built-in ones; the cycle
 * is skipped while a plugin term is not ready().
 *
 * input_stamps(): one per term, in `observation_names` order. A state store term has the
 * oldest stamp of its entries, a plugin term the stamp it returns. `actions` and the motion
 * command fed back from the outputs derive from the previous observation, so they have its
 * stamp (the oldest of its other terms; 0 before the first step after clear()).
 *
 * If the model has a `time_step` input it receives the number of steps since reset(); the
 * motion command is re-seeded from the robot state after reset().
 *
 * Parameters (under `<policy>.`): `imu_name` (default `imu`), `velocity_command_name`
 * (default `cmd_vel`), `odometry_name` (default `odom`), `observation_input` (default `obs`).
 */
class MjlabPreprocessor : public policy_framework::Preprocessor
{
public:
  /// @throws std::runtime_error on missing metadata, unknown term or size mismatch.
  void configure(
    const policy_framework::PluginContext & ctx, const policy_framework::ModelInfo & model,
    policy_framework::DataStore & state) override;
  void clear() override;
  void reset() override;
  bool compute(
    const policy_framework::DataStore & state, const policy_framework::TensorMap & last_outputs,
    policy_framework::TensorMap & inputs) override;
  /// Pushes the current values into the history, once it was started by a compute().
  void observe(
    const policy_framework::DataStore & state,
    const policy_framework::TensorMap & last_outputs) override;
  const policy_framework::InputStamps & input_stamps() const override {return stamps_;}

  /// @return Number of past observations per term in the model input. Valid after configure().
  size_t history_length() const {return history_length_;}

private:
  struct Term
  {
    std::shared_ptr<policy_framework::ObservationTerm> term;  ///< Built-in or plugin.
    size_t dim = 0;
    std::vector<float> history;  ///< history_length * dim values, oldest first.
  };

  /// @return true when every input of every term is available.
  bool ready(const policy_framework::DataStore & state) const;
  /// Append the current value of every term to its history (and record the stamps).
  void push(
    const policy_framework::DataStore & state, const policy_framework::TensorMap & last_outputs);

  // Declared before terms_ so that it outlives the plugin instances.
  std::unique_ptr<pluginlib::ClassLoader<policy_framework::ObservationTerm>> term_loader_;
  std::vector<Term> terms_;
  size_t history_length_ = 1;
  std::string obs_input_;
  bool has_time_step_ = false;
  // Control-cycle state.
  bool history_filled_ = false;
  policy_framework::InputStamps stamps_;
  double prev_obs_stamp_ = 0.0;  ///< Oldest input stamp of the previous step, 0 after clear().
  int step_ = 0;  ///< Steps since reset().
};

}  // namespace policy_framework_mjlab

#endif  // POLICY_FRAMEWORK_MJLAB__MJLAB_PREPROCESSOR_HPP_
