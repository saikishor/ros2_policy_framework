// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__HOSTS__POLICY_REPLAY_HPP_
#define POLICY_FRAMEWORK__HOSTS__POLICY_REPLAY_HPP_

#include <memory>
#include <string>

#include "policy_framework/core/step_replayer.hpp"

namespace policy_framework
{

/**
 * @brief Run every policy step recorded in a bag through @p inference (see StepReplayer).
 * @param bag Path of the bag (any storage rosbag2 reads, e.g. MCAP).
 * @param topic Topic of the policy_framework/msg/PolicyStep messages; empty: the only one of
 *        that type in the bag.
 * @param inference A configured model.
 * @param policy Replay only the steps of this policy; empty: every step.
 * @throws std::runtime_error when the bag has no such topic, or several PolicyStep topics and
 *         none was named, or a step does not match the model.
 */
StepReplayer::Summary replay_bag(
  const std::string & bag, const std::string & topic, std::shared_ptr<Inference> inference,
  const std::string & policy = "");

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__HOSTS__POLICY_REPLAY_HPP_
