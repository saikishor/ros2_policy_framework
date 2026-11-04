// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__PLUGIN_INTERFACES_HPP_
#define POLICY_FRAMEWORK__PLUGIN_INTERFACES_HPP_

#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "policy_framework/data_store.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

/**
 * @file plugin_interfaces.hpp
 * @brief Base classes of every pluginlib extension point of the framework.
 *
 * One cycle of a policy is:
 *
 *   InputSource(s) --> state DataStore --> Preprocessor --> TensorMap --> Inference
 *     --> TensorMap --> Postprocessor --> command DataStore --> Executor(s)
 *
 * Input sources and executors are the transport and are shared by all policies: in the
 * standalone node, topic Executors; in the controller, InterfaceExecutors writing command
 * interfaces (state interfaces are read by the controller itself). Preprocessor, inference and
 * postprocessor are owned by one policy and do not see ROS transport, so the same policy runs
 * unchanged in the standalone node and in the controller. The exception is a topic
 * observation term (observation_term.hpp), for data that only exists on a topic.
 */

namespace policy_framework
{

/// Dense float32 tensor in row-major order.
struct Tensor
{
  std::vector<int64_t> shape;
  std::vector<float> data;
};

/// Tensors keyed by model input/output name.
using TensorMap = std::map<std::string, Tensor>;

/// Static description of a loaded model, produced by the Inference stage.
struct ModelInfo
{
  /// Custom model metadata (e.g. ONNX `metadata_props`).
  std::map<std::string, std::string> metadata;
  /// One zero-filled tensor per model input, with its (static) shape.
  TensorMap inputs;
  /// One zero-filled tensor per model output, with its (static) shape.
  TensorMap outputs;
};

/**
 * @brief When the data of one input of a policy step was produced.
 *
 * Stamps are seconds on the node clock: the message header stamp for topic data (the receive
 * time when the header is empty or the message has none), the cycle time for state
 * interfaces (ros2_control interfaces carry no stamp). 0 means unknown (e.g. never received).
 * They let a host measure how old each input is and the latency down to the commands.
 */
struct InputStamp
{
  std::string name;
  double stamp = 0.0;
  /// The input is fed back from the policy's previous outputs (e.g. the last action) and has
  /// the stamp of the previous observation; it does not count for the observation stamp.
  bool from_previous_outputs = false;
};
using InputStamps = std::vector<InputStamp>;

class Kinematics;

/// What a plugin needs to configure itself.
struct PluginContext
{
  /// Node of the runner (standalone node or controller), used for parameters, topics, timers.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  /// Name of the policy, source, executor or term; plugin parameters live under `<name>.`.
  std::string name;
  /// Whole-body kinematics (null unless `kinematics.enabled`), updated by the host before every
  /// policy step; given to the policy stages and their observation terms only.
  std::shared_ptr<Kinematics> kinematics = nullptr;
};

/**
 * @brief Declare (when not declared yet) and return the plugin parameter `<ctx.name>.<param>`.
 *
 * Several policies can share a plugin type, and the node and the controller both re-run
 * configuration, hence the "declare once" semantics.
 */
template<typename T>
T get_parameter(const PluginContext & ctx, const std::string & param, const T & default_value)
{
  const auto full = ctx.name + "." + param;
  if (!ctx.node->has_parameter(full)) {ctx.node->declare_parameter<T>(full, default_value);}
  return ctx.node->get_parameter(full).template get_value<T>();
}

/**
 * @brief Runs the model. The only framework-specific stage (ONNX, TensorRT, ...).
 */
class Inference
{
public:
  virtual ~Inference() = default;
  /**
   * @brief Load the model.
   * @throws std::exception on any configuration or loading error.
   */
  virtual void configure(const PluginContext & ctx) = 0;
  /// @return Description of the loaded model. Valid after configure().
  virtual const ModelInfo & model() const = 0;
  /**
   * @brief Run one inference step. Called from the control cycle.
   * @param inputs Tensors with the shapes of model().inputs.
   * @param outputs Tensors with the shapes of model().outputs; overwritten in place.
   * @return false on failure.
   */
  virtual bool run(const TensorMap & inputs, TensorMap & outputs) = 0;
};

/**
 * @brief Builds the model inputs from the state store (observation terms, history, ...).
 */
class Preprocessor
{
public:
  virtual ~Preprocessor() = default;
  /**
   * @brief Configure and register in @p state every entry this preprocessor reads.
   * @throws std::exception on any configuration error.
   */
  virtual void configure(const PluginContext & ctx, const ModelInfo & model, DataStore & state) = 0;
  /**
   * @brief Forget everything observed so far (e.g. the observation history). Called before
   *        the first activation and when the host (re)starts, never in between.
   */
  virtual void clear() {}
  /**
   * @brief Called whenever the policy becomes active (after clear() if any), before the next
   *        compute(). Reset per-activation state here (e.g. a motion time step); state that
   *        should survive a switch away and back (e.g. history) belongs to clear().
   */
  virtual void reset() {}
  /**
   * @brief Fill the model inputs. Called from the control cycle.
   * @param state Latest values of all inputs.
   * @param last_outputs Model outputs of the previous step of this policy (zeros after
   *        clear()), e.g. to feed back the last action.
   * @param inputs Tensors to fill, shaped as ModelInfo::inputs.
   * @return false when the required data is not available yet (the cycle is skipped).
   */
  virtual bool compute(
    const DataStore & state, const TensorMap & last_outputs, TensorMap & inputs) = 0;
  /**
   * @return One stamp per input of the last successful compute() (e.g. per observation term
   *         or per entry). The names and their order are fixed after configure(); only the
   *         stamps change. Empty when the preprocessor does not report stamps.
   */
  virtual const InputStamps & input_stamps() const
  {
    static const InputStamps none;
    return none;
  }
};

/**
 * @brief Converts the model outputs into named commands (scaling, offsets, limits, ...).
 */
class Postprocessor
{
public:
  virtual ~Postprocessor() = default;
  /**
   * @brief Configure and register in @p commands every entry this postprocessor writes.
   * @throws std::exception on any configuration error.
   */
  virtual void configure(
    const PluginContext & ctx, const ModelInfo & model, DataStore & commands) = 0;
  /// Called from the control cycle whenever the policy becomes active. Must be real-time safe.
  virtual void reset() {}
  /**
   * @brief Write the commands. Called from the control cycle.
   * @param stamp Stamp of the observation the outputs were computed from (oldest input, see
   *        PolicyPipeline::observation_stamp()); write it as the stamp of every command.
   * @return false on failure.
   */
  virtual bool compute(const TensorMap & outputs, double stamp, DataStore & commands) = 0;
};

/**
 * @brief Feeds the state store from a non ros2_control source (typically a topic).
 *
 * Callbacks of the source must not touch the store; they buffer the data (e.g. in a
 * realtime_tools buffer) and read() copies it into the store from the control cycle.
 */
class InputSource
{
public:
  virtual ~InputSource() = default;
  /// @throws std::exception on any configuration error.
  virtual void configure(const PluginContext & ctx) = 0;
  /**
   * @return true when this source produces the entry @p name. Used to decide which entries
   *         the controller has to claim as state interfaces and to validate the node setup.
   */
  virtual bool provides(const std::string & name) const = 0;
  /**
   * @brief Copy the latest data into the registered entries of @p state.
   * @param now Current time in seconds.
   */
  virtual void read(DataStore & state, double now) = 0;
};

/**
 * @brief Sends the commands somewhere other than ros2_control command interfaces (typically a
 *        topic).
 */
class Executor
{
public:
  virtual ~Executor() = default;
  /**
   * @brief Configure. @p commands already holds every entry the policies may write.
   * @throws std::exception on any configuration error.
   */
  virtual void configure(const PluginContext & ctx, const DataStore & commands) = 0;
  /// Send the commands. Called from the control cycle.
  virtual void write(const DataStore & commands) = 0;
};

/**
 * @brief Turns the commands into ros2_control command interface values, inside
 *        PolicyController (the controller's counterpart of the topic Executor, which only
 *        runs in the standalone node).
 *
 * It may compute (e.g. a PD law into efforts) and runs on every controller update, also
 * between policy steps when the policy runs at a lower rate, with the state interfaces read
 * in that update. It does not touch loaned interfaces: it fills values that the controller
 * writes, so it can be tested without a controller manager.
 */
class InterfaceExecutor
{
public:
  virtual ~InterfaceExecutor() = default;
  /**
   * @brief Configure. @p commands already holds every entry the policies may write.
   * @param state Register every state entry read in write(); the controller claims them as
   *        state interfaces, read on every update.
   * @throws std::exception on any configuration error.
   */
  virtual void configure(const PluginContext & ctx, const DataStore & commands, DataStore & state) = 0;
  /// @return Full names of the command interfaces to claim. Valid after configure().
  virtual const std::vector<std::string> & command_interfaces() const = 0;
  /**
   * @return State entries read in write() that must be fresh on every update (e.g. joint
   *         feedback of a PD law); configuration fails if an input source provides one.
   */
  virtual std::vector<std::string> state_entries() const {return {};}
  /**
   * @brief Compute the interface values. Called from the control loop on every update, once
   *        the active policy has produced commands since the controller was activated.
   * @param values One per command_interfaces(); NaN leaves that interface unchanged.
   */
  virtual void write(const DataStore & commands, const DataStore & state, std::vector<double> & values) = 0;
  /**
   * @brief Values written when the controller stops commanding: on deactivation and when the
   *        policy fails. Default: NaN (leave every interface unchanged).
   */
  virtual void safe_values(std::vector<double> & values)
  {
    values.assign(values.size(), std::numeric_limits<double>::quiet_NaN());
  }
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__PLUGIN_INTERFACES_HPP_
