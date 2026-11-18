// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

#ifndef POLICY_FRAMEWORK__DATA_STORE_HPP_
#define POLICY_FRAMEWORK__DATA_STORE_HPP_

#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace policy_framework
{

/**
 * @brief Flat table of named scalar values shared between the pipeline stages.
 *
 * It is the transport-agnostic contract of the framework: input sources (topics or ros2_control
 * state interfaces) write into one store and preprocessors read from it; postprocessors write
 * commands into another store that executors (topics or command interfaces) read.
 *
 * Names follow the ros2_control `<prefix>/<interface>` convention (e.g. `joint1/position`,
 * `imu/orientation.x`), so moving a value from a topic to a state interface is pure
 * configuration.
 *
 * Entries are registered with add() while configuring (allocates); afterwards the real-time
 * path only uses the index based get()/set() accessors (no allocation, no hashing).
 * The store is not thread-safe: only the thread running the pipeline cycle may touch it.
 */
class DataStore
{
public:
  /**
   * @brief Register an entry (idempotent).
   * @param name Entry name.
   * @return Index of the entry, stable for the lifetime of the store.
   */
  size_t add(const std::string & name)
  {
    const auto [it, inserted] = index_.try_emplace(name, names_.size());
    if (inserted) {
      names_.push_back(name);
      values_.push_back(0.0);
      stamps_.push_back(0.0);
      received_.push_back(0);
    }
    return it->second;
  }

  /**
   * @brief Copy the values, stamps and received flags of @p other, which must have the same
   *        entries (e.g. a copy of this store). Does not allocate: real-time safe, used to hand
   *        data between the control loop and an inference thread.
   */
  void copy_values_from(const DataStore & other)
  {
    std::copy(other.values_.begin(), other.values_.end(), values_.begin());
    std::copy(other.stamps_.begin(), other.stamps_.end(), stamps_.begin());
    std::copy(other.received_.begin(), other.received_.end(), received_.begin());
  }

  /**
   * @brief Forget every value: all entries are zero and not received again, as right after
   *        add(). Does not allocate.
   */
  void reset()
  {
    std::fill(values_.begin(), values_.end(), 0.0);
    std::fill(stamps_.begin(), stamps_.end(), 0.0);
    std::fill(received_.begin(), received_.end(), 0);
  }

  /// @return Index of @p name, or std::nullopt when it was never registered.
  [[nodiscard]] std::optional<size_t> find(const std::string & name) const
  {
    const auto it = index_.find(name);
    return it == index_.end() ? std::nullopt : std::optional<size_t>(it->second);
  }

  /**
   * @return Index of @p name.
   * @throws std::out_of_range when @p name was never registered.
   */
  [[nodiscard]] size_t index(const std::string & name) const
  {
    const auto it = index_.find(name);
    if (it == index_.end()) {
      throw std::out_of_range("DataStore: unknown entry '" + name + "'");
    }
    return it->second;
  }

  /**
   * @brief Write a value and mark the entry as received.
   * @param i Entry index.
   * @param value New value.
   * @param stamp Time the value was produced, in seconds.
   */
  void set(size_t i, double value, double stamp)
  {
    values_[i] = value;
    stamps_[i] = stamp;
    received_[i] = 1;
  }

  [[nodiscard]] double get(size_t i) const {return values_[i];}
  /// @return Stamp in seconds of the last set() of entry @p i.
  [[nodiscard]] double stamp(size_t i) const {return stamps_[i];}
  /// @return true once entry @p i has been set() at least once.
  [[nodiscard]] bool received(size_t i) const {return received_[i] != 0;}

  /// @return true when every index in @p indices has been received.
  [[nodiscard]] bool all_received(const std::vector<size_t> & indices) const
  {
    for (const auto i : indices) {
      if (!received_[i]) {return false;}
    }
    return true;
  }

  [[nodiscard]] size_t size() const {return names_.size();}
  [[nodiscard]] const std::vector<std::string> & names() const {return names_;}

private:
  std::unordered_map<std::string, size_t> index_;
  std::vector<std::string> names_;
  std::vector<double> values_;
  std::vector<double> stamps_;
  // uint8_t instead of bool: std::vector<bool> is a bitset with proxy references.
  std::vector<uint8_t> received_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__DATA_STORE_HPP_
