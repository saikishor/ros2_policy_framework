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


#ifndef POLICY_FRAMEWORK__PLUGIN_INTERFACES_HPP_
#define POLICY_FRAMEWORK__PLUGIN_INTERFACES_HPP_

#include "policy_framework/interfaces/types.hpp"
#include "policy_framework/interfaces/plugin_context.hpp"
#include "policy_framework/interfaces/inference.hpp"
#include "policy_framework/interfaces/preprocessor.hpp"
#include "policy_framework/interfaces/postprocessor.hpp"
#include "policy_framework/interfaces/input_source.hpp"
#include "policy_framework/interfaces/executor.hpp"
#include "policy_framework/interfaces/interface_executor.hpp"
#include "policy_framework/interfaces/safety_check.hpp"

/**
 * @file plugin_interfaces.hpp
 * @brief Every pluginlib extension point of the framework, one header each in interfaces/.
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

#endif  // POLICY_FRAMEWORK__PLUGIN_INTERFACES_HPP_
