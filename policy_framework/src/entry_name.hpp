// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef ENTRY_NAME_HPP_
#define ENTRY_NAME_HPP_

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

#endif  // ENTRY_NAME_HPP_
