// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__SAFETY_CHECKS_HPP_
#define POLICY_FRAMEWORK__SAFETY_CHECKS_HPP_

#include <string>
#include <vector>

#include "policy_framework/plugin_interfaces.hpp"

namespace policy_framework
{

/**
 * @brief SafetyCheck that keeps state entries `<prefix>/<interface>` within bounds, e.g. motor
 *        temperatures below a limit or actuator error codes at 0.
 *
 * Parameters (under `<name>.`):
 * - `prefixes` (string array): e.g. joint or actuator names
 * - `interfaces` (string array): e.g. `temperature`, `error_code`; one entry per prefix each
 * - `<interface>.min`, `<interface>.max` (double, default unbounded): at least one of them
 * - `<interface>.hysteresis` (double, default 0): once an entry failed, it passes again only
 *   within [min + hysteresis, max - hysteresis], so that a value hovering at a bound does not
 *   toggle the check (e.g. fail at 80 degrees, pass again below 70)
 * - `trigger_cycles` (int, default 1): consecutive violations of an entry before it fails, to
 *   ride out a single noisy reading
 *
 * A non-finite value is a violation; an entry never received is not checked.
 */
class StateBoundsCheck : public SafetyCheck
{
public:
  /// @throws std::runtime_error when an interface has neither `min` nor `max`, or when its
  ///         hysteresis leaves no value to pass again.
  void configure(const PluginContext & ctx, const DataStore & commands, DataStore & state) override;
  Verdict check(const DataStore & state, double now, std::string & reason) override;
  void reset() override;

private:
  struct Entry
  {
    size_t index;
    double min;
    double max;
    double hysteresis;
    int violations = 0;  ///< In a row.
    bool failed = false;  ///< Until the value is back within the bounds minus the hysteresis.
  };
  std::vector<Entry> entries_;
  int trigger_cycles_ = 1;
};

/**
 * @brief SafetyCheck that fails when a state entry `<prefix>/<interface>` changes more than
 *        `<interface>.max_delta` between two cycles, e.g. an encoder glitch.
 *
 * Parameters (under `<name>.`):
 * - `prefixes`, `interfaces` (string arrays): the entries, as for StateBoundsCheck
 * - `<interface>.max_delta` (double, required, > 0): largest change between two cycles
 *
 * The first value received after configure() or reset() is never a jump. A non-finite value is
 * a jump. A wrapping value (the position of a continuous joint reported in [-pi, pi]) jumps by
 * 2 pi at the wrap: do not check those.
 */
class JumpCheck : public SafetyCheck
{
public:
  /// @throws std::runtime_error when an interface has no `max_delta` above zero.
  void configure(const PluginContext & ctx, const DataStore & commands, DataStore & state) override;
  Verdict check(const DataStore & state, double now, std::string & reason) override;
  void reset() override;

private:
  struct Entry
  {
    size_t index;
    double max_delta;
    double previous = 0.0;
    bool has_previous = false;
  };
  std::vector<Entry> entries_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__SAFETY_CHECKS_HPP_
