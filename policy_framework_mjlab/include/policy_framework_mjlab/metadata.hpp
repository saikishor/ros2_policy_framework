// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK_MJLAB__METADATA_HPP_
#define POLICY_FRAMEWORK_MJLAB__METADATA_HPP_

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace policy_framework_mjlab
{

/// Split a comma separated mjlab metadata value.
inline std::vector<std::string> split(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string token;
  while (std::getline(ss, token, ',')) {out.push_back(token);}
  return out;
}

/// Parse a comma separated list of numbers. @throws std::invalid_argument on bad numbers.
inline std::vector<double> parse_doubles(const std::string & s)
{
  std::vector<double> out;
  for (const auto & t : split(s)) {out.push_back(std::stod(t));}
  return out;
}

/// @return metadata[key]. @throws std::runtime_error when missing.
inline const std::string & require(
  const std::map<std::string, std::string> & metadata, const std::string & key)
{
  const auto it = metadata.find(key);
  if (it == metadata.end()) {
    throw std::runtime_error("Model metadata has no '" + key + "'");
  }
  return it->second;
}

/// Metadata `joint_names` and their `default_joint_pos`.
struct Joints
{
  std::vector<std::string> names;
  std::vector<double> defaults;
};

/// @return The model joints. @throws std::runtime_error when missing or the sizes differ.
inline Joints joints(const std::map<std::string, std::string> & metadata)
{
  Joints j{split(require(metadata, "joint_names")),
    parse_doubles(require(metadata, "default_joint_pos"))};
  if (j.defaults.size() != j.names.size()) {
    throw std::runtime_error("default_joint_pos and joint_names sizes differ");
  }
  return j;
}

/**
 * @return The joints listed in metadata[key] (a subset of `joint_names`, e.g. pal_mjlab's
 *         `observation_joint_names`) with their defaults; every model joint when there is no
 *         such key.
 * @throws std::runtime_error when a listed joint is not a model joint.
 */
inline Joints joints(const std::map<std::string, std::string> & metadata, const std::string & key)
{
  const auto all = joints(metadata);
  const auto it = metadata.find(key);
  if (it == metadata.end()) {return all;}
  Joints subset;
  for (const auto & name : split(it->second)) {
    const auto pos = std::find(all.names.begin(), all.names.end(), name);
    if (pos == all.names.end()) {
      throw std::runtime_error("Model metadata '" + key + "' has unknown joint '" + name + "'");
    }
    subset.names.push_back(name);
    subset.defaults.push_back(all.defaults[static_cast<size_t>(pos - all.names.begin())]);
  }
  return subset;
}

/**
 * @return metadata[key] as @p size counts (pal_mjlab writes them as "3.000").
 * @throws std::runtime_error on another size or a value that is not a positive integer.
 */
inline std::vector<size_t> parse_counts(
  const std::map<std::string, std::string> & metadata, const std::string & key, size_t size)
{
  std::vector<size_t> out;
  for (const double v : parse_doubles(require(metadata, key))) {
    if (v < 1.0 || std::trunc(v) < v) {
      throw std::runtime_error("Model metadata '" + key + "' must hold positive integers");
    }
    out.push_back(static_cast<size_t>(v));
  }
  if (out.size() != size) {
    throw std::runtime_error(
            "Model metadata '" + key + "' has " + std::to_string(out.size()) + " values, expected " +
            std::to_string(size));
  }
  return out;
}

}  // namespace policy_framework_mjlab

#endif  // POLICY_FRAMEWORK_MJLAB__METADATA_HPP_
