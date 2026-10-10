// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK_MJLAB__MJLAB_PREPROCESSOR_HPP_
#define POLICY_FRAMEWORK_MJLAB__MJLAB_PREPROCESSOR_HPP_

#include <string>
#include <vector>

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
 * - `imu_projected_gravity`: from `<imu_name>/orientation.{x,y,z,w}`
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
 * value. The history is kept when the policy is switched away and back (reset() only), like
 * pal_policy_deployer: back-filling a single frame of a dynamic state (e.g. a jump landing)
 * made history-3 policies fall in simulation.
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

  /// @return Number of past observations per term in the model input. Valid after configure().
  size_t history_length() const {return history_length_;}

private:
  enum class Kind
  {
    BASE_ANG_VEL, BASE_LIN_ACC, PROJECTED_GRAVITY, BASE_LIN_VEL, JOINT_POS, JOINT_VEL,
    ACTIONS, TWIST_COMMAND, MOTION_COMMAND
  };
  struct Term
  {
    Kind kind;
    size_t dim;
    std::vector<float> history;  ///< history_length * dim values, oldest first.
  };

  /// Write the current value of @p term into @p out (term.dim values).
  void current_value(
    const Term & term, const policy_framework::DataStore & state,
    const policy_framework::TensorMap & last_outputs, float * out) const;

  std::vector<Term> terms_;
  size_t history_length_ = 1;
  std::string obs_input_;
  bool has_time_step_ = false;
  std::vector<double> default_joint_pos_;
  // Store indices.
  std::vector<size_t> required_;
  std::vector<size_t> joint_pos_, joint_vel_, ang_vel_, lin_acc_, quat_, lin_vel_, twist_;
  // Control-cycle state.
  std::vector<float> scratch_;
  bool history_filled_ = false;
  bool first_step_ = true;  ///< First step since reset().
  int step_ = 0;
};

}  // namespace policy_framework_mjlab

#endif  // POLICY_FRAMEWORK_MJLAB__MJLAB_PREPROCESSOR_HPP_
