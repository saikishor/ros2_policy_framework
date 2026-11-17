// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__CORE__COMMAND_BLENDER_HPP_
#define POLICY_FRAMEWORK__CORE__COMMAND_BLENDER_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

#include "policy_framework/core/data_store.hpp"

namespace policy_framework
{

/**
 * @brief Ramps the commands the executors read from where they are to the commands of a
 *        policy, so that a policy starting or taking over does not step the references.
 *
 * It owns a copy of the command store, output(): start() remembers its current values, and
 * every update() moves it linearly from them to the target (the policy's commands) over the
 * duration given to start(). Once the ramp is over, and whenever no ramp was started, output()
 * equals the target. Every received entry is blended (references and gains alike); an entry
 * the output never held takes the target at once, unless set_start() gave it a start value
 * (e.g. the measured position of a joint nothing commanded yet).
 *
 * Only start(), set_start(), update() and reset() run in the control cycle; none allocates.
 */
class CommandBlender
{
public:
  /// Give output() the entries of @p commands. Allocates: call while configuring.
  void configure(const DataStore & commands)
  {
    output_ = commands;
    output_.reset();
    start_.assign(commands.size(), 0.0);
    has_start_.assign(commands.size(), 0);
    duration_ = 0.0;
    elapsed_ = 0.0;
  }

  /// Forget the output and any ramp, as before the first command.
  void reset()
  {
    output_.reset();
    std::fill(has_start_.begin(), has_start_.end(), 0);
    duration_ = 0.0;
    elapsed_ = 0.0;
  }

  /**
   * @brief Start a ramp from the current output to the target of the following update()s.
   * @param duration Seconds; <= 0: no ramp, the output follows the target at once.
   */
  void start(double duration)
  {
    for (size_t i = 0; i < start_.size(); ++i) {
      has_start_[i] = output_.received(i) ? 1 : 0;
      start_[i] = output_.get(i);
    }
    duration_ = std::max(duration, 0.0);
    elapsed_ = 0.0;
  }

  /// @return true when the ramp started last has a start value for entry @p i.
  [[nodiscard]] bool has_start(size_t i) const {return has_start_[i] != 0;}

  /// Make the ramp of entry @p i start from @p value. Call after start().
  void set_start(size_t i, double value)
  {
    start_[i] = value;
    has_start_[i] = 1;
  }

  /**
   * @brief Advance the ramp by @p dt seconds and compute output() from @p target, which must
   *        have the entries given to configure(). Entries @p target never received are left
   *        as they are.
   */
  void update(const DataStore & target, double dt)
  {
    elapsed_ = std::min(elapsed_ + dt, duration_);
    const double alpha = duration_ > 0.0 ? elapsed_ / duration_ : 1.0;
    for (size_t i = 0; i < start_.size(); ++i) {
      if (!target.received(i)) {continue;}
      const double t = target.get(i);
      const double v = alpha < 1.0 && has_start_[i] ? start_[i] + alpha * (t - start_[i]) : t;
      output_.set(i, v, target.stamp(i));
    }
  }

  /// @return true while a ramp is in progress.
  [[nodiscard]] bool blending() const {return elapsed_ < duration_;}

  [[nodiscard]] const DataStore & output() const {return output_;}

private:
  DataStore output_;
  std::vector<double> start_;
  // uint8_t instead of bool, as in DataStore.
  std::vector<uint8_t> has_start_;
  double duration_ = 0.0;
  double elapsed_ = 0.0;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__CORE__COMMAND_BLENDER_HPP_
