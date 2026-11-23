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

#ifndef POLICY_FRAMEWORK__CORE__SAFETY_MONITOR_HPP_
#define POLICY_FRAMEWORK__CORE__SAFETY_MONITOR_HPP_

#include <bitset>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "policy_framework/core/data_store.hpp"

namespace policy_framework
{

/**
 * @brief Checks the measured state against limits, to stop following a policy that drives the
 *        robot somewhere it must not be (a fall, a joint out of its limits, a dead input).
 *
 * Every check is off by default. check() returns the first violated check. Values that were
 * never received are not checked: a missing input is the policy's concern (it waits for it),
 * not a violation. configure() registers the state entries the enabled checks read.
 *
 * PolicyRunner runs it from the `safety.*` parameters (see PolicyRunner::update_safety()).
 */
class SafetyMonitor
{
public:
  struct Config
  {
    /// Largest angle (rad) between the IMU z axis and the world z axis; 0: off.
    double max_tilt = 0.0;
    /// State entry prefix of the IMU orientation, `<imu>/orientation.{x,y,z,w}`.
    std::string imu;
    /// Position limits (lower, upper) per joint; only commanded joints are checked.
    std::map<std::string, std::pair<double, double>> joint_limits;
    /// The joint positions must stay this far inside joint_limits.
    double position_margin = 0.0;
    /// Largest age (s) of the observation of the last policy step; 0: off.
    double max_input_age = 0.0;
  };

  enum class Check {TILT, JOINT_POSITION, INPUT_AGE};
  /// A set of checks, one bit per Check.
  using Checks = std::bitset<3>;
  /// @return The bit of @p check in Checks.
  static constexpr size_t bit(Check check) {return static_cast<size_t>(check);}

  /// A violated check.
  struct Violation
  {
    Check check;
    /// Measured value: tilt (rad), joint position, observation age (s).
    double value;
    /// The joint, for the joint checks.
    std::string joint;
    /// Human readable description. Allocates: for logging the fault, not every cycle.
    std::string describe() const;
  };

  /**
   * @brief Register the state entries read by the enabled checks. The commanded joints are
   *        those with a `<joint>/position` entry in @p commands.
   * @throws std::invalid_argument when max_tilt is set without an imu.
   */
  void configure(const Config & config, const DataStore & commands, DataStore & state);

  /**
   * @return The first violated check, or nothing. Does not allocate unless a check is
   *         violated (the joint name is copied).
   * @param observation_stamp Stamp of the observation of the last policy step (0: none yet).
   * @param now Current time, on the clock of the stamps.
   * @param skipped Checks not evaluated (e.g. disabled by the active policy).
   */
  std::optional<Violation> check(
    const DataStore & state, double observation_stamp, double now,
    const Checks & skipped = {}) const;

  /**
   * @return Position limits (lower, upper) of every single-dof joint of the URDF @p urdf that
   *         has finite ones (continuous joints have none).
   * @throws std::exception when the URDF cannot be parsed.
   */
  static std::map<std::string, std::pair<double, double>> joint_limits(const std::string & urdf);

private:
  struct Joint
  {
    std::string name;
    size_t index;
    double lower = 0.0;
    double upper = 0.0;
  };
  Config config_;
  std::vector<size_t> orientation_;  ///< x, y, z, w; empty: tilt not checked.
  std::vector<Joint> positions_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__CORE__SAFETY_MONITOR_HPP_
