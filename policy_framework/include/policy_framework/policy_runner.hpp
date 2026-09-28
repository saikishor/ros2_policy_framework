// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__POLICY_RUNNER_HPP_
#define POLICY_FRAMEWORK__POLICY_RUNNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "policy_framework/data_store.hpp"
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
 *
 * A cycle is read_sources() → [host reads state interfaces] → step() → [host writes command
 * interfaces] → write_executors() (only after a successful step()).
 */
class PolicyRunner
{
public:
  /**
   * @brief Load and configure every plugin, and create the `~/switch_policy` service.
   * @throws std::exception on any configuration error (missing parameter, plugin, model...).
   */
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /// @return State entries required by the policies that no input source provides.
  std::vector<std::string> unprovided_state() const;

  /// Copy the latest data of every input source into state(). @p now in seconds.
  void read_sources(double now);

  /// Run one step of the active policy. Never throws; exceptions are reported as ERROR.
  PolicyPipeline::Status step();

  /// Send commands() through every executor.
  void write_executors();

  DataStore & state() {return state_;}
  DataStore & commands() {return commands_;}
  PolicyPipeline & pipeline() {return *pipeline_;}

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  // Loaders must outlive the plugin instances created from them (declared first, destroyed
  // last).
  std::unique_ptr<pluginlib::ClassLoader<InputSource>> source_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Preprocessor>> preprocessor_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Inference>> inference_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Postprocessor>> postprocessor_loader_;
  std::unique_ptr<pluginlib::ClassLoader<Executor>> executor_loader_;

  std::vector<std::shared_ptr<InputSource>> sources_;
  std::vector<std::shared_ptr<Executor>> executors_;
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
