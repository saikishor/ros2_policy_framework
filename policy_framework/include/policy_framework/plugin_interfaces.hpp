// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__PLUGIN_INTERFACES_HPP_
#define POLICY_FRAMEWORK__PLUGIN_INTERFACES_HPP_

#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
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
  /// The input is a user command (a velocity command, a motion reference): held until its own
  /// timeout, so it is old by design; it does not count for the observation stamp.
  bool user_command = false;
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
  /// Whole-body kinematics (null unless `kinematics.enabled`) of the state the policy step
  /// reads, updated before every step; given to the policy stages and their observation terms
  /// only.
  std::shared_ptr<Kinematics> kinematics = nullptr;
  /// State entry that is 0 while user commands must be ignored (see CommandGate); empty: none.
  std::string safety_entry = "";
};

/**
 * @brief State entry that the host sets above 0 while a fault with the NEUTRAL_COMMANDS
 *        reaction is active (see CommandGate). Registered and written by the runner itself, so
 *        neither an input source nor a state interface provides it.
 */
inline constexpr char kNeutralCommandsEntry[] = "policy_framework/neutral_commands";

/**
 * @brief Whether user commands (a velocity command, a motion reference...) may be used now.
 *
 * With the host parameter `safety_entry` set (e.g. `rl_safety_controller/safe`, a state
 * interface in the controller), commands are only accepted while that state entry was received
 * and is greater than 0. Independently of it, commands are refused while the host sets
 * kNeutralCommandsEntry above 0 (a NEUTRAL_COMMANDS fault reaction), and accepted again as soon
 * as it clears it. Plugins that feed user commands to a policy check open() and use their
 * neutral command (e.g. zero velocity) while it is false; the policy itself keeps running.
 */
class CommandGate
{
public:
  CommandGate() = default;
  explicit CommandGate(const PluginContext & ctx)
  : name_(ctx.safety_entry) {}

  /// Real-time safe after the first call (which looks the entries up in @p state).
  bool open(const DataStore & state)
  {
    if (!resolved_) {
      if (!name_.empty()) {index_ = state.find(name_);}
      neutral_index_ = state.find(kNeutralCommandsEntry);
      resolved_ = true;
    }
    if (neutral_index_ && state.received(*neutral_index_) && state.get(*neutral_index_) > 0.0) {
      return false;
    }
    if (name_.empty()) {return true;}
    return index_ && state.received(*index_) && state.get(*index_) > 0.0;
  }

private:
  std::string name_;
  std::optional<size_t> index_;
  std::optional<size_t> neutral_index_;
  bool resolved_ = false;
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
   * @brief Called from the control cycle on every step while another policy is active, instead
   *        of compute(). Keep time-dependent memory current here (e.g. the observation
   *        history), so that the policy does not resume from a stale past when switched back.
   * @param last_outputs Model outputs of the last step of this policy.
   */
  virtual void observe(const DataStore & state, const TensorMap & last_outputs)
  {
    (void)state;
    (void)last_outputs;
  }

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
   * @brief Write the commands that are already known before the first step, e.g. the policy's
   *        gains; not references (the executors hold the robot until the first step, see
   *        InterfaceExecutor::hold()). Called for the default policy when the host starts,
   *        outside the control cycle.
   */
  virtual void initial_commands(DataStore & commands) {(void)commands;}
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
 * @brief What a host does about a fault (a failed safety check, a failed policy step...).
 *
 * Ordered from the most control kept to the least: a host only ever escalates to a later
 * reaction. NEUTRAL_COMMANDS lifts as soon as no fault asks for it any more; every other
 * reaction is latched until the host is reactivated. PolicyRunner::update_safety() applies
 * them; policy_node stops its commands on a latched one, PolicyController makes its executors
 * apply it (InterfaceExecutor::fault()) or returns an error.
 */
enum class FaultReaction
{
  /// The policy keeps running, with neutral user commands (e.g. zero velocity, see
  /// CommandGate): the executors keep following it.
  NEUTRAL_COMMANDS,
  HOLD,   ///< Keep the pose of the fault, e.g. a PD law around it.
  DAMP,   ///< Let the robot down gently, e.g. a pure joint damping.
  SAFE,   ///< The safe values, e.g. zero effort.
  ERROR   ///< Stop: the host returns an error and is deactivated (fallback controllers take over).
};

/**
 * @return The reaction named @p name in a configuration (`neutral_commands`, `hold`, `damp`,
 *         `safe`, `error`), or nothing for any other name.
 */
inline std::optional<FaultReaction> fault_reaction_from_string(const std::string & name)
{
  if (name == "neutral_commands") {return FaultReaction::NEUTRAL_COMMANDS;}
  if (name == "hold") {return FaultReaction::HOLD;}
  if (name == "damp") {return FaultReaction::DAMP;}
  if (name == "safe") {return FaultReaction::SAFE;}
  if (name == "error") {return FaultReaction::ERROR;}
  return std::nullopt;
}

/// @return The configuration name of @p reaction (see fault_reaction_from_string()).
inline const char * to_string(FaultReaction reaction)
{
  switch (reaction) {
    case FaultReaction::NEUTRAL_COMMANDS: return "neutral_commands";
    case FaultReaction::HOLD: return "hold";
    case FaultReaction::DAMP: return "damp";
    case FaultReaction::SAFE: return "safe";
    case FaultReaction::ERROR: return "error";
  }
  return "unknown";
}

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
   *        the active policy has produced commands since the controller was activated (until
   *        then: hold()).
   * @param values One per command_interfaces(); NaN leaves that interface unchanged.
   */
  virtual void write(const DataStore & commands, const DataStore & state, std::vector<double> & values) = 0;
  /**
   * @brief Compute the interface values that keep the robot where it is. Called from the
   *        control loop on every update from the activation of the controller until the active
   *        policy produced its first commands (e.g. while it waits for its inputs, or for its
   *        first inference), instead of write().
   * @param commands Only holds the policy's Postprocessor::initial_commands().
   * @param start true on the first call after an activation.
   * @param values One per command_interfaces(); NaN leaves that interface unchanged. Default:
   *        all NaN.
   */
  virtual void hold(
    const DataStore & commands, const DataStore & state, std::vector<double> & values, bool start)
  {
    (void)commands;
    (void)state;
    (void)start;
    values.assign(values.size(), std::numeric_limits<double>::quiet_NaN());
  }

  /**
   * @brief Values written when the controller stops commanding: on deactivation and when the
   *        policy fails. Default: NaN (leave every interface unchanged).
   */
  virtual void safe_values(std::vector<double> & values)
  {
    values.assign(values.size(), std::numeric_limits<double>::quiet_NaN());
  }

  /**
   * @return true when fault() can apply @p reaction with this configuration (e.g. DAMP needs a
   *         damping). Meant for hosts to refuse, while configuring, a reaction that their
   *         configuration uses but an executor cannot apply. Default: NEUTRAL_COMMANDS (the
   *         executor keeps following the policy), SAFE and ERROR.
   */
  virtual bool supports_fault_reaction(FaultReaction reaction) const
  {
    return reaction == FaultReaction::NEUTRAL_COMMANDS || reaction == FaultReaction::SAFE ||
           reaction == FaultReaction::ERROR;
  }

  /**
   * @brief Values written on every update while a host is faulted with an internal reaction
   *        (HOLD, DAMP or SAFE; with NEUTRAL_COMMANDS the host keeps calling write(), with
   *        ERROR it is deactivated, which writes safe_values()). Called from the control loop,
   *        only with a supported reaction.
   *        Default: safe_values().
   * @param commands Latest command entries, e.g. the gains the policy wrote.
   * @param state Latest values of the state entries registered in configure().
   * @param reaction The reaction of the fault.
   * @param start true on the first call with this @p reaction, e.g. to capture the pose to hold.
   */
  virtual void fault(
    const DataStore & commands, const DataStore & state, std::vector<double> & values,
    FaultReaction reaction, bool start)
  {
    (void)commands;
    (void)state;
    (void)reaction;
    (void)start;
    safe_values(values);
  }
};

/**
 * @brief A check of the measured state that tells whether the robot is in a state it must not
 *        be in (e.g. an actuator reporting an error, a motor too hot, the centre of mass outside
 *        the feet).
 *
 * The contract: configure() registers every state entry the check reads, check() is then
 * called with the state on every cycle of the control loop and returns a Verdict that is not
 * safe when the state is unsafe, and reset() forgets what the check accumulated (filters,
 * counters).
 *
 * What to do about a failure is configured per instance (`on_fault`), not decided by the check;
 * a failing check may ask for another reaction in its Verdict, among the ones it declares once
 * configured (possible_reactions()), e.g. a more severe one when several actuators fail.
 *
 * PolicyRunner loads them from its `safety_checks` parameter (see PolicyRunner::update_safety()).
 */
class SafetyCheck
{
public:
  /// The outcome of check(): no allocation, so that it can be returned from the control loop.
  struct Verdict
  {
    bool safe = true;
    /// When not safe: the reaction asked for, one of possible_reactions(); nothing: the
    /// `on_fault` configured for this check.
    std::optional<FaultReaction> reaction;
  };

  virtual ~SafetyCheck() = default;

  /**
   * @brief Register the state entries the check reads (state.add()).
   * @param ctx Parameters live under `<ctx.name>.`; ctx.kinematics is a whole-body model of the
   *        state (null when there is none).
   * @param commands Command entries of the policies (e.g. to find the commanded joints).
   * @param state Store the checked entries are registered in.
   * @throws std::exception on invalid parameters.
   */
  virtual void configure(
    const PluginContext & ctx, const DataStore & commands, DataStore & state) = 0;

  /**
   * @brief Check the state. Runs in the control loop: must not allocate while it passes.
   * @param state The store given to configure(), with the current values.
   * @param now Current time in seconds, on the clock of the state stamps.
   * @param[out] reason Why it failed; written only on failure (so that passing never
   *             allocates).
   * @return Whether the state is safe and, when it is not, the reaction asked for (e.g. a more
   *         severe one when several actuators fail at once).
   */
  virtual Verdict check(const DataStore & state, double now, std::string & reason) = 0;

  /**
   * @return The reactions a Verdict may ask for, besides the `on_fault` configured for this
   *         check, known once configured: hosts refuse while configuring one that an executor
   *         does not support. Default: none.
   */
  virtual std::vector<FaultReaction> possible_reactions() const {return {};}

  /// Forget filters and counters, so that a new run does not inherit the previous one's.
  virtual void reset() {}
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__PLUGIN_INTERFACES_HPP_
