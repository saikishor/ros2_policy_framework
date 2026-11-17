// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__POLICY_RUNNER_HPP_
#define POLICY_FRAMEWORK__POLICY_RUNNER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "policy_framework/data_store.hpp"
#include "policy_framework/kinematics.hpp"
#include "policy_framework/plugin_interfaces.hpp"
#include "policy_framework/policy_pipeline.hpp"
#include "policy_framework/srv/switch_policy.hpp"
#include "policy_framework/step_publisher.hpp"

namespace policy_framework
{

/**
 * @brief Everything a policy host (standalone node or ros2_control controller) needs: loads the
 *        plugins from parameters, owns the stores, serves `~/switch_policy` and runs the cycle.
 *
 * Parameters:
 * - `sources` (string array): input source names; each needs `<source>.type` (plugin class).
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
 * - `executors` (string array): executor names; each needs `<executor>.type` (plugin class).
 * - `publish_steps` (bool, default false): publish every policy step on `~/policy_step`, see
 *   StepPublisher.
 * - `kinematics.enabled`, `kinematics.joints`: optional whole-body Kinematics model, fed from
 *   the `<joint>/position` and `<joint>/velocity` state entries (registered here). The host
 *   updates kinematics() with update_kinematics(). The policies' plugins get
 *   policy_kinematics() in PluginContext::kinematics, which always matches the state their step
 *   reads: the same model when the step runs in the host's thread, a second one updated by
 *   step() from its own state when it runs in another thread.
 *
 * A cycle: [controller: copy state interfaces] → read_sources() → update_kinematics() → step()
 * → node: write_executors() / controller: its InterfaceExecutors (after a successful step()).
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
   *        and write its initial commands. Hosts call it when they are activated.
   */
  void restart()
  {
    pipeline_->restart();
    pipeline_->write_initial_commands(commands_);
  }

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

  /// Send commands() through every executor.
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
  std::unique_ptr<StepPublisher> step_publisher_;  ///< Null unless `publish_steps`.

  // Used only from step(), to log transitions once instead of every cycle.
  CommandGate command_gate_;
  bool commands_accepted_ = true;
  PolicyPipeline::Status last_status_ = PolicyPipeline::Status::OK;
  std::string last_policy_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__POLICY_RUNNER_HPP_
