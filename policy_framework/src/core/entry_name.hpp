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

#ifndef POLICY_FRAMEWORK__SRC__CORE__ENTRY_NAME_HPP_
#define POLICY_FRAMEWORK__SRC__CORE__ENTRY_NAME_HPP_

#include <optional>
#include <string>

namespace policy_framework
{

/// @return `<prefix>` when @p name is `<prefix><suffix>` with a non-empty prefix (internal).
inline std::optional<std::string> strip_suffix(const std::string & name, const std::string & suffix)
{
  if (name.size() <= suffix.size() ||
    name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0)
  {
    return std::nullopt;
  }
  return name.substr(0, name.size() - suffix.size());
}

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__SRC__CORE__ENTRY_NAME_HPP_
