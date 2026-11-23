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

#ifndef POLICY_FRAMEWORK__OBSERVATION_TERM_HPP_
#define POLICY_FRAMEWORK__OBSERVATION_TERM_HPP_

#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "policy_framework/latest_message.hpp"
#include "policy_framework/plugin_interfaces.hpp"

namespace policy_framework
{

/**
 * @brief One observation term (a block of floats of the current frame), as a plugin.
 *
 * Lets a preprocessor (e.g. MjlabPreprocessor) gain terms without modifying it; the
 * preprocessor keeps the history and the layout, a term only writes its current value.
 *
 * A term reads its data from the state store (register the entries in configure(); use this
 * for data that also exists as state interfaces, so a controller can read them in real time)
 * or directly from a topic (derive from TopicTerm; for data with no state interface equivalent,
 * such as a velocity command or a custom message).
 */
class ObservationTerm
{
public:
  virtual ~ObservationTerm() = default;

  /// Returned by compute() for a value derived from the policy's previous outputs (e.g. the
  /// last action): it has no stamp of its own and does not count for the observation stamp.
  static constexpr double kFromPreviousOutputs = -1.0;
  /**
   * @brief Configure; register in @p state every entry read.
   * @param ctx Parameters of the term live under `<ctx.name>.`.
   * @return Number of floats of the term (> 0).
   * @throws std::exception on any configuration error.
   */
  virtual size_t configure(const PluginContext & ctx, const ModelInfo & model, DataStore & state) = 0;
  /**
   * @return false while the data is not available yet; the preprocessor then skips the cycle.
   *         Called from the control cycle.
   */
  virtual bool ready(const DataStore & state) = 0;
  /// Called whenever the policy becomes active. Real-time safe.
  virtual void reset() {}
  /**
   * @return true for a user command term (derived from CommandTerm): its stamp does not count
   *         for the observation stamp, since a command is held until its own timeout and is old
   *         by design.
   */
  bool user_command() const {return user_command_;}
  /**
   * @return When the latest message of the term arrived (seconds, node clock), for a term with
   *         its own subscription (TopicTerm); 0 otherwise or before any. Real-time safe.
   */
  virtual double receive_time() {return 0.0;}
  /// @return The term's `max_age` (seconds, 0: off), see TopicTerm.
  double max_age() const {return max_age_;}
  /// @return The reaction when receive_time() is older than max_age().
  FaultReaction max_age_on_fault() const {return max_age_on_fault_;}
  /**
   * @brief Write the current value. Called from the control cycle, only when ready().
   * @param last_outputs Model outputs of the previous step of the policy.
   * @param out The configure()d number of floats.
   * @return Stamp of the written value (seconds, node clock): when the data was produced, i.e.
   *         the message header stamp; the current time for a message without one, and for a
   *         value generated now (e.g. a default). Read it from the same message as the value.
   *         kFromPreviousOutputs for a value taken from @p last_outputs.
   */
  virtual double compute(const DataStore & state, const TensorMap & last_outputs, float * out) = 0;

protected:
  /// Set by CommandTerm only.
  bool user_command_ = false;
  /// Set by TopicTerm only.
  double max_age_ = 0.0;
  FaultReaction max_age_on_fault_ = FaultReaction::ERROR;
};

/**
 * @brief Observation term fed directly by a topic (parameter `topic`, default given by the
 *        derived class). Ready once a message arrived; override ready() otherwise.
 */
template<typename MsgT>
class TopicTerm : public ObservationTerm
{
public:
  explicit TopicTerm(std::string default_topic) : default_topic_(std::move(default_topic)) {}

  /**
   * Parameters (under the term's name): `topic`; `max_age` (seconds, default 0: off): the host
   * faults with `max_age_on_fault` (hold, damp, safe or error; default error) when the latest
   * message arrived longer ago, checked after each policy step. Not for a CommandTerm, whose
   * timeout handles an operator who stopped sending.
   * @throws std::runtime_error on an invalid `max_age_on_fault`, or a `max_age` on a CommandTerm.
   */
  size_t configure(const PluginContext & ctx, const ModelInfo & model, DataStore &) override
  {
    this->max_age_ = get_parameter<double>(ctx, "max_age", 0.0);
    const auto reaction = fault_reaction_from_string(
      get_parameter<std::string>(ctx, "max_age_on_fault", "error"));
    if (!reaction || *reaction == FaultReaction::NEUTRAL_COMMANDS) {
      // Neutral commands keep the policy running: on the stale data.
      throw std::runtime_error(ctx.name + ".max_age_on_fault must be hold, damp, safe or error");
    }
    this->max_age_on_fault_ = *reaction;
    if (this->max_age_ > 0.0 && this->user_command_) {
      throw std::runtime_error(
              ctx.name + ".max_age: a command term has a timeout instead (its staleness is normal)");
    }
    const auto dim = configure_term(ctx, model);
    topic_.subscribe(ctx.node, topic_parameter(ctx, default_topic_));
    return dim;
  }

  bool ready(const DataStore &) override {return latest().msg != nullptr;}
  double receive_time() final {return latest().receive_time;}

protected:
  using Received = typename LatestMessage<MsgT>::Received;

  /**
   * @brief Read the term parameters and check the model.
   * @return Number of floats of the term.
   */
  virtual size_t configure_term(const PluginContext & ctx, const ModelInfo & model) = 0;

  /// @return The latest message (null until the first one) and when it arrived. Real-time safe.
  Received latest() {return topic_.get();}

  /// @return Current time of the node clock (the receive_time clock), seconds.
  double now() const {return topic_.now();}

private:
  std::string default_topic_;
  LatestMessage<MsgT> topic_;
};

namespace detail
{
template<typename MsgT, typename = void>
struct has_header : std::false_type {};
template<typename MsgT>
struct has_header<MsgT, std::void_t<decltype(std::declval<MsgT>().header.stamp)>>
  : std::true_type {};
}  // namespace detail

/**
 * @brief An observation term fed by a user command topic (a velocity command, a motion
 *        reference), with everything a command needs to be safe handled here, so that a command
 *        term cannot get it wrong:
 *
 * - the neutral command (write_neutral()) before the first message, once the last one was
 *   received longer ago than `timeout` (parameter, seconds, above zero: a command that never
 *   expired would stay applied after the operator is lost), while usable() refuses it, and
 *   while the CommandGate is closed (the host's `safety_entry` is 0, or the NEUTRAL_COMMANDS
 *   fault reaction is active);
 * - the stamp: the header stamp of the message when its type has a header and it is set, else
 *   its receive time (so that its age is how long ago it arrived); the current time for the
 *   neutral command;
 * - the user command flag: its stamp does not count for the observation stamp.
 *
 * Always ready by default: a policy does not wait for a command (a term may override ready()
 * to wait for something else, e.g. the base orientation). Parameters
 * (under the term's name): `topic`, `timeout`.
 *
 * A derived term implements configure_command(), write_command() and write_neutral(), and may
 * refuse a message in usable(). On every cycle exactly one of write_command() and
 * write_neutral() is called, so write_neutral() runs on every cycle while the command is
 * neutral, not once on the change: a term with state (e.g. the alignment of a stream) resets it
 * there. These three run in the control cycle: no allocation, blocking or logging on every
 * call.
 *
 * The gate reads the host's `safety_entry` from the PluginContext given to configure(): the
 * preprocessors pass the host's; a term configured with a context without it (e.g. in a test)
 * has a gate that only the NEUTRAL_COMMANDS reaction closes.
 */
template<typename MsgT>
class CommandTerm : public TopicTerm<MsgT>
{
public:
  /**
   * @param default_topic Topic when the `topic` parameter is not set.
   * @param default_timeout Seconds, when the `timeout` parameter is not set.
   */
  CommandTerm(std::string default_topic, double default_timeout)
  : TopicTerm<MsgT>(std::move(default_topic)), default_timeout_(default_timeout)
  {
    this->user_command_ = true;
  }

  bool ready(const DataStore &) override {return true;}

  /**
   * @brief Write the command, or the neutral command, and return its stamp. The command is
   *        used when a message was received at most `timeout` seconds ago (on the receive
   *        time), the CommandGate is open and usable() accepts it, stamped with its header
   *        stamp, else its receive time; otherwise the neutral command, stamped now.
   */
  double compute(const DataStore & state, const TensorMap &, float * out) final
  {
    const auto in = this->latest();
    const double t = this->now();
    // Measured on the receive time: whatever clock the publisher stamps with.
    if (in.msg && t - in.receive_time <= timeout_ && gate_.open(state) && usable(*in.msg)) {
      write_command(*in.msg, state, out);
      return stamp_of(*in.msg, in.receive_time);
    }
    write_neutral(state, out);
    return t;
  }

protected:
  /**
   * @brief Read the term parameters and check the model.
   * @return Number of floats of the term.
   */
  virtual size_t configure_command(const PluginContext & ctx, const ModelInfo & model) = 0;
  /**
   * @return false to use the neutral command instead of @p msg (e.g. a malformed message).
   *         Called on every cycle with the latest message, often the same one again: cache
   *         what is costly to check per message (e.g. map the joints of a message only when
   *         they change), and throttle any log.
   */
  virtual bool usable(const MsgT &) {return true;}
  /// Write the term from the command @p msg. Called from the control cycle.
  virtual void write_command(const MsgT & msg, const DataStore & state, float * out) = 0;
  /// Write the neutral command (e.g. zero velocity, a still stand). Called from the control cycle.
  virtual void write_neutral(const DataStore & state, float * out) = 0;

  size_t configure_term(const PluginContext & ctx, const ModelInfo & model) final
  {
    timeout_ = get_parameter<double>(ctx, "timeout", default_timeout_);
    if (!(timeout_ > 0.0)) {
      throw std::runtime_error(ctx.name + ".timeout must be above zero");
    }
    gate_ = CommandGate(ctx);
    return configure_command(ctx, model);
  }

private:
  static double stamp_of(const MsgT & msg, double receive_time)
  {
    if constexpr (detail::has_header<MsgT>::value) {
      const auto & s = msg.header.stamp;
      if (s.sec != 0 || s.nanosec != 0) {return s.sec + s.nanosec * 1e-9;}
    }
    return receive_time;
  }

  double default_timeout_;
  double timeout_ = 0.0;
  CommandGate gate_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__OBSERVATION_TERM_HPP_
