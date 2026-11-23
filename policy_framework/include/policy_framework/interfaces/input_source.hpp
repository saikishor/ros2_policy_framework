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


#ifndef POLICY_FRAMEWORK__INTERFACES__INPUT_SOURCE_HPP_
#define POLICY_FRAMEWORK__INTERFACES__INPUT_SOURCE_HPP_

#include <string>

#include "policy_framework/core/data_store.hpp"
#include "policy_framework/interfaces/plugin_context.hpp"

/// @file input_source.hpp
/// @brief Base class of the InputSource plugins.

namespace policy_framework
{

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
  /**
   * @return When the latest data arrived (seconds, node clock), 0 before any. The host faults
   *         when it is older than the source's `max_age` (see PolicyRunner). Real-time safe.
   */
  virtual double receive_time() = 0;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__INTERFACES__INPUT_SOURCE_HPP_
