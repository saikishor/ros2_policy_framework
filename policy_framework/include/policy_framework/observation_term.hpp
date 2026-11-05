// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__OBSERVATION_TERM_HPP_
#define POLICY_FRAMEWORK__OBSERVATION_TERM_HPP_

#include <string>
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
   * @brief Write the current value. Called from the control cycle, only when ready().
   * @param last_outputs Model outputs of the previous step of the policy.
   * @param out The configure()d number of floats.
   * @return Stamp of the written value (seconds, node clock): when the data was produced,
   *         e.g. the message header stamp, else its receive time; the current time for a
   *         value generated now (e.g. a default). Read it from the same message as the value.
   */
  virtual double compute(const DataStore & state, const TensorMap & last_outputs, float * out) = 0;
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

  size_t configure(const PluginContext & ctx, const ModelInfo & model, DataStore &) override
  {
    const auto dim = configure_term(ctx, model);
    topic_.subscribe(ctx.node, topic_parameter(ctx, default_topic_));
    return dim;
  }

  bool ready(const DataStore &) override {return latest().msg != nullptr;}

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

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__OBSERVATION_TERM_HPP_
