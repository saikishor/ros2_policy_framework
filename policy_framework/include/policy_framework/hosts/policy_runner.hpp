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

#ifndef POLICY_FRAMEWORK__HOSTS__POLICY_RUNNER_HPP_
#define POLICY_FRAMEWORK__HOSTS__POLICY_RUNNER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "policy_framework/core/command_blender.hpp"
#include "policy_framework/core/data_store.hpp"
#include "policy_framework/core/kinematics.hpp"
#include "policy_framework/plugin_interfaces.hpp"
#include "policy_framework/core/policy_pipeline.hpp"
#include "policy_framework/core/safety_monitor.hpp"
#include "policy_framework/srv/switch_policy.hpp"

namespace policy_runner
{
struct Params;  // generated from the parameters, kept out of this header
}  // namespace policy_runner

namespace policy_framework
{

/**
 * @brief Everything a policy host (standalone node or ros2_control controller) needs: loads the
 *        plugins from parameters, owns the stores, serves `~/switch_policy` and runs the cycle.
 *
 * Parameters:
 * - `sources` (string array): input source names; each needs `<source>.type` (plugin class),
 *   and may have `<source>.max_age` (seconds, default 0: off): a fault, with the reaction
 *   `<source>.max_age_on_fault` (default `error`), when its latest message arrived longer ago.
 * - `policies` (string array, required): policy names; the first one is active at start.
 *   Per policy:
 *   - `<policy>.preprocessor` (plugin class, required)
 *   - `<policy>.inference` (plugin class, default `policy_framework/OnnxInference`)
 *   - `<policy>.postprocessor` (plugin class, required)
 *   - `<policy>.max_steps` (int, default 0): > 0 returns to the first policy after N steps.
 *   - `<policy>.interruptible` (bool, default true)
 *   plus the parameters of each of its plugins, also under `<policy>.`.
 * - `safety_entry` (string, default empty): state entry that is 0 while user commands must be
 *   ignored (see CommandGate), registered here and handed to the plugins.
 * - `safety.*`: the checks of a SafetyMonitor, all off by default (`max_tilt` with `imu`,
 *   `joint_position_limits` with `position_margin`, `max_input_age`), each with its reaction
 *   (`tilt_on_fault`, `joint_position_on_fault`, `input_age_on_fault`, default `error`; see
 *   FaultReaction for the names and their order).
 * - `safety_checks` (string array): SafetyCheck plugin names; each needs `<check>.type`, has a
 *   reaction `<check>.on_fault` (default `error`), plus the parameters of the plugin under
 *   `<check>.`. There is no global default: every check states its reaction.
 * - `on_policy_failure` (string, default `error`): reaction to a failed policy step (hold, damp,
 *   safe or error).
 * - `executors` (string array): executor names; each needs `<executor>.type` (plugin class).
 * - `kinematics.enabled`, `kinematics.joints`: optional whole-body Kinematics model, fed from
 *   the `<joint>/position` and `<joint>/velocity` state entries (registered here). The host
 *   updates kinematics() with update_kinematics(). The policies' plugins get
 *   policy_kinematics() in PluginContext::kinematics, which always matches the state their step
 *   reads: the same model when the step runs in the host's thread, a second one updated by
 *   step() from its own state when it runs in another thread.
 *
 * A cycle: [controller: copy state interfaces] → read_sources() → update_kinematics() →
 * update_safety() → step() (unless a latched reaction is active) → check_input_ages() (after a
 * successful step) → node: write_executors() / controller: its InterfaceExecutors.
 */
class PolicyRunner
{
public:
  /**
   * @brief Load and configure every plugin, and create the `~/switch_policy` service.
   * @param load_executors Load the `executors` as topic Executor plugins. The controller
   *        passes false and loads them itself as InterfaceExecutor plugins.
   * @param robot_description URDF of the robot; required only by the kinematics.
   * @param step_in_other_thread step() is called from another thread than update_kinematics(),
   *        on a copy of state() (e.g. the controller's inference thread).
   * @throws std::exception on any configuration error (missing parameter, plugin, model...).
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, bool load_executors = true,
    const std::string & robot_description = "", bool step_in_other_thread = false);

  /// @return State entries required by the policies that no input source provides.
  std::vector<std::string> unprovided_state() const;

  /**
   * @brief Start over from the default policy with every policy cleared (at the next step()),
   *        and write its initial commands; forget executor_commands(). Hosts call it when they
   *        are activated (allocates).
   */
  void restart();

  /**
   * @brief Run the `safety.*` checks and every `safety_checks` plugin on state(), and apply the
   *        reaction of the most severe failing one. Hosts call it on every cycle, after
   *        refreshing state() and the kinematics and before step().
   *
   * Checks the active policy lists in `disabled_safety_checks` are skipped (not run), and reset
   * when they apply again, so that they start afresh; a latched reaction stays latched.
   * NEUTRAL_COMMANDS lasts while a failing check asks for it (set_neutral_commands()); every
   * other reaction is latched until restart(), and only ever escalates. Changes are logged.
   * Real-time safe while every check passes; a failing check builds its reason.
   * @param observation_stamp Stamp of the observation of the last successful step (0: none).
   * @param now Current time in seconds, on the clock of the state stamps.
   * @return The reaction in effect: nothing when every check passes and nothing is latched.
   */
  std::optional<FaultReaction> update_safety(double observation_stamp, double now);

  /**
   * @brief Latch a failed policy step (including a non-finite model output) with
   *        `on_policy_failure`.
   * @return The reaction in effect.
   */
  FaultReaction policy_failure(const std::string & reason);

  /**
   * @brief Check the inputs of the step that just finished against their `max_age` (see
   *        TopicTerm), and latch the reaction of the most severe stale one. Hosts call it right
   *        after a successful step, in their own thread (with an inference thread: when they
   *        collect the step's result, before the next step starts).
   * @param now Current time in seconds, on the clock of the receive times.
   * @return The latched reaction in effect, if any.
   */
  std::optional<FaultReaction> check_input_ages(double now);

  /**
   * @return The index in @p stamps of the stale input (received longer ago than its max_age)
   *         with the most severe reaction; nothing when none is. An input that never received
   *         anything, or without a max_age, is not checked.
   */
  static std::optional<size_t> stale_input(const InputStamps & stamps, double now);

  /// The input names of every policy (from its preprocessor's input_stamps()), by policy.
  struct PolicyInputs
  {
    std::string policy;
    std::vector<std::string> inputs;
  };
  /// @return The inputs of every policy, in the order they were configured.
  const std::vector<PolicyInputs> & policy_inputs() const {return policy_inputs_;}

  /// @return Every reaction the configuration can lead to, for hosts to check their support.
  std::vector<FaultReaction> fault_reactions() const;

  /// @return Whether any `safety.*` check or `safety_checks` plugin is configured.
  bool has_safety_checks() const
  {
    return monitor_enabled_ || !checks_.empty() || !source_ages_.empty();
  }

  /**
   * @return The reaction of a failed @p verdict: the one it asks for when @p declared has it,
   *         ERROR when it does not, @p configured when it asks for none.
   */
  static FaultReaction reaction_of(
    const SafetyCheck::Verdict & verdict, FaultReaction configured,
    const std::vector<FaultReaction> & declared);

  /**
   * @brief Make the user command plugins use their neutral command (@p on true) or accept the
   *        commands again (see CommandGate), e.g. while a fault with the NEUTRAL_COMMANDS
   *        reaction is active. Writes kNeutralCommandsEntry in state(); real-time safe.
   * @param now Stamp of the entry, in seconds.
   */
  void set_neutral_commands(bool on, double now) {state_.set(neutral_idx_, on ? 1.0 : 0.0, now);}

  /// Copy the latest data of every input source into state(). @p now in seconds.
  void read_sources(double now);

  /**
   * @brief Update kinematics() (if any) from the joint positions and velocities of state(),
   *        once all of them were received. Real-time safe. Hosts call it after refreshing
   *        state(), at least once per policy step and before it (the controller: every update).
   */
  void update_kinematics();

  /// Run one step of the active policy. Never throws; exceptions are reported as ERROR.
  PolicyPipeline::Status step() {return step(state_, commands_);}
  /**
   * @brief The same on other stores with the layout of state() and commands() (copies of
   *        them), e.g. owned by an inference thread. Call from one thread at a time.
   */
  PolicyPipeline::Status step(const DataStore & state, DataStore & commands);

  /**
   * @brief Compute executor_commands() from commands(), ramping them over the policy's
   *        `transition_time` when a new policy took over. Real-time safe.
   * @param stepped commands() hold the result of a step that just finished (synchronous: this
   *        cycle's step; asynchronous: the step whose commands were just collected). A ramp
   *        starts when that step's policy is not the one of the previous ramp: on the first
   *        step after restart() (from the measured `<joint>/position` state entries) and after
   *        a switch.
   * @param dt Seconds since the previous call (the controller calls it on every update).
   */
  void blend_commands(bool stepped, double dt);
  /// @return The commands the executors read: commands() through the transition ramp.
  const DataStore & executor_commands() const {return blender_.output();}

  /// Send executor_commands() through every executor.
  void write_executors();

  /// @return (name, plugin class) of each configured `executors` entry, loaded or not.
  const std::vector<std::pair<std::string, std::string>> & executor_types() const
  {
    return executor_types_;
  }

  /// @return The kinematics of the latest state(), null unless `kinematics.enabled`.
  const std::shared_ptr<Kinematics> & kinematics() const {return kinematics_.model;}
  /// @return The kinematics of the state of the current policy step, given to the plugins.
  const std::shared_ptr<Kinematics> & policy_kinematics() const {return policy_kinematics_.model;}

  DataStore & state() {return state_;}
  DataStore & commands() {return commands_;}
  PolicyPipeline & pipeline() {return *pipeline_;}

private:
  /// A kinematics model with the buffers to feed it from a store, one per feeding thread.
  struct KinematicsFeed
  {
    std::shared_ptr<Kinematics> model;
    Eigen::VectorXd q, v;
  };
  /// Create the kinematics and register their state entries when `kinematics.enabled`.
  void configure_kinematics(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & robot_description, bool step_in_other_thread);
  /// Update @p feed from the joint positions and velocities of @p state, once all were received.
  void update_kinematics(KinematicsFeed & feed, const DataStore & state) const;

  // Not the node itself: PolicyNode owns its runner, a node pointer here would be a cycle.
  rclcpp::Logger logger_ = rclcpp::get_logger("policy_runner");
  // Loaders must outlive the plugin instances created from them (declared first, destroyed
  // last).
  std::unique_ptr<pluginlib::ClassLoader<InputSource>> source_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Preprocessor>> preprocessor_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Inference>> inference_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Postprocessor>> postprocessor_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Executor>> executor_loader_;
  std::unique_ptr<pluginlib::ClassLoader<SafetyCheck>> safety_check_loader_;

  KinematicsFeed kinematics_;         ///< Of state(), fed by update_kinematics().
  KinematicsFeed policy_kinematics_;  ///< Of the step's state; may share kinematics_'s model.
  // State indices of the kinematics joints, in joint_names() order.
  std::vector<size_t> kinematics_position_idx_;
  std::vector<size_t> kinematics_velocity_idx_;
  std::vector<std::shared_ptr<InputSource>> sources_;
  std::vector<std::shared_ptr<Executor>> executors_;
  std::vector<std::pair<std::string, std::string>> executor_types_;
  // unique_ptr: the pipeline holds atomics, so it is replaced rather than reassigned.
  std::unique_ptr<PolicyPipeline> pipeline_ = std::make_unique<PolicyPipeline>();
  DataStore state_;
  DataStore commands_;
  rclcpp::Service<srv::SwitchPolicy>::SharedPtr switch_service_;
  CommandBlender blender_;
  int blend_policy_ = -1;  ///< Policy of the last ramp; -1: none since restart().
  /// (command index, state index) of the `<joint>/position` commands that are measured.
  std::vector<std::pair<size_t, size_t>> measured_positions_;

  /// A loaded `safety_checks` plugin.
  struct Check
  {
    std::string name;
    std::shared_ptr<SafetyCheck> check;
    FaultReaction on_fault;
    std::vector<FaultReaction> declared;  ///< Its possible_reactions().
    bool skipped = false;  ///< Disabled by the policy active at the previous update_safety().
  };
  /// The checks a policy disables (`<policy>.disabled_safety_checks`).
  struct PolicySafety
  {
    SafetyMonitor::Checks monitor;
    std::vector<bool> checks;  ///< Per checks_ entry.
  };
  /// Load the monitor and the `safety_checks` plugins, after the policies.
  void configure_safety(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const policy_runner::Params & params,
    const std::string & robot_description);
  /// Escalate latched_ to @p reaction, never back.
  void escalate(FaultReaction reaction);
  /// Log a change of the reaction in effect, caused by @p reason.
  void report(std::optional<FaultReaction> effective, const std::string & reason);

  SafetyMonitor monitor_;
  bool monitor_enabled_ = false;
  FaultReaction policy_failure_reaction_ = FaultReaction::ERROR;
  FaultReaction tilt_on_fault_ = FaultReaction::ERROR;
  FaultReaction position_on_fault_ = FaultReaction::ERROR;
  FaultReaction input_age_on_fault_ = FaultReaction::ERROR;
  std::vector<FaultReaction> monitor_reactions_;  ///< Of the enabled monitor checks.
  std::vector<Check> checks_;
  std::vector<PolicySafety> policy_safety_;  ///< Per policy, in the pipeline order.
  /// A source with a `max_age`.
  struct SourceAge
  {
    std::string name;
    std::shared_ptr<InputSource> source;
    double max_age;
    FaultReaction on_fault;
  };
  std::vector<SourceAge> source_ages_;
  std::vector<FaultReaction> input_age_reactions_;  ///< Of the inputs with a max_age.
  std::vector<PolicyInputs> policy_inputs_;
  std::optional<FaultReaction> latched_;   ///< Latched reaction, until restart().
  std::optional<FaultReaction> reported_;  ///< Reaction in effect when last logged.
  std::string reason_;        ///< Of the most severe failing check of the current cycle.
  std::string check_reason_;  ///< Written by a failing check.

  // Used only from step(), to log transitions once instead of every cycle.
  CommandGate command_gate_;
  size_t neutral_idx_ = 0;  ///< kNeutralCommandsEntry in state_.
  bool commands_accepted_ = true;
  PolicyPipeline::Status last_status_ = PolicyPipeline::Status::OK;
  std::string last_policy_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__HOSTS__POLICY_RUNNER_HPP_
