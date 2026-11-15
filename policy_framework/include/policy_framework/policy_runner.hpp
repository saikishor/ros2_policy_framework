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
 * - `executors` (string array): executor names; each needs `<executor>.type` (plugin class).
 * - `kinematics.enabled`, `kinematics.joints`: optional whole-body Kinematics model, handed to
 *   the policies' plugins in PluginContext::kinematics and updated by update_kinematics() from the
 *   `<joint>/position` and `<joint>/velocity` state entries (registered here).
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
   * @throws std::exception on any configuration error (missing parameter, plugin, model...).
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, bool load_executors = true,
    const std::string & robot_description = "");

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
   * @brief Update the kinematics (if any) from the joint positions and velocities of state(),
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

  /// @return The kinematics model, null unless `kinematics.enabled`.
  const std::shared_ptr<Kinematics> & kinematics() const {return kinematics_;}

  DataStore & state() {return state_;}
  DataStore & commands() {return commands_;}
  PolicyPipeline & pipeline() {return *pipeline_;}

private:
  /// Create kinematics_ and register its state entries when `kinematics.enabled`.
  void configure_kinematics(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & robot_description);

  // Not the node itself: PolicyNode owns its runner, a node pointer here would be a cycle.
  rclcpp::Logger logger_ = rclcpp::get_logger("policy_runner");
  // Loaders must outlive the plugin instances created from them (declared first, destroyed
  // last).
  std::unique_ptr<pluginlib::ClassLoader<InputSource>> source_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Preprocessor>> preprocessor_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Inference>> inference_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Postprocessor>> postprocessor_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Executor>> executor_loader_;

  std::shared_ptr<Kinematics> kinematics_;
  // State indices and buffers of the kinematics joints, in kinematics_->joint_names() order.
  std::vector<size_t> kinematics_position_idx_;
  std::vector<size_t> kinematics_velocity_idx_;
  Eigen::VectorXd kinematics_q_;
  Eigen::VectorXd kinematics_v_;
  std::vector<std::shared_ptr<InputSource>> sources_;
  std::vector<std::shared_ptr<Executor>> executors_;
  std::vector<std::pair<std::string, std::string>> executor_types_;
  // unique_ptr: the pipeline holds atomics, so it is replaced rather than reassigned.
  std::unique_ptr<PolicyPipeline> pipeline_ = std::make_unique<PolicyPipeline>();
  DataStore state_;
  DataStore commands_;
  rclcpp::Service<srv::SwitchPolicy>::SharedPtr switch_service_;

  // Used only from step(), to log transitions once instead of every cycle.
  PolicyPipeline::Status last_status_ = PolicyPipeline::Status::OK;
  std::string last_policy_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__POLICY_RUNNER_HPP_
