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


#ifndef POLICY_FRAMEWORK__INTERFACES__EXECUTOR_HPP_
#define POLICY_FRAMEWORK__INTERFACES__EXECUTOR_HPP_

#include "policy_framework/core/data_store.hpp"
#include "policy_framework/interfaces/plugin_context.hpp"

/// @file executor.hpp
/// @brief Base class of the topic Executor plugins.

namespace policy_framework
{

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

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__INTERFACES__EXECUTOR_HPP_
