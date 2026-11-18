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
 * - `trigger_cycles` (int, default 1): consecutive violations of an entry before it fails, to
 *   ride out a single noisy reading
 *
 * A non-finite value is a violation; an entry never received is not checked.
 */
class StateBoundsCheck : public SafetyCheck
{
public:
  /// @throws std::runtime_error when an interface has neither `min` nor `max`.
  void configure(const PluginContext & ctx, const DataStore & commands, DataStore & state) override;
  bool check(const DataStore & state, double now, std::string & reason) override;
  void reset() override;

private:
  struct Entry
  {
    size_t index;
    double min;
    double max;
    int violations = 0;  ///< In a row.
  };
  std::vector<Entry> entries_;
  int trigger_cycles_ = 1;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__SAFETY_CHECKS_HPP_
