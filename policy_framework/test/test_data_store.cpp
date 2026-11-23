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

#include <gtest/gtest.h>

#include "policy_framework/data_store.hpp"

using policy_framework::DataStore;

TEST(DataStore, AddIsIdempotentAndReturnsStableIndex)
{
  DataStore store;
  const auto a = store.add("joint1/position");
  const auto b = store.add("joint1/velocity");
  EXPECT_NE(a, b);
  EXPECT_EQ(store.add("joint1/position"), a);
  EXPECT_EQ(store.size(), 2u);
  EXPECT_EQ(store.names()[b], "joint1/velocity");
}

TEST(DataStore, FindReturnsEmptyForUnknownNames)
{
  DataStore store;
  store.add("a");
  EXPECT_TRUE(store.find("a").has_value());
  EXPECT_FALSE(store.find("b").has_value());
  EXPECT_THROW((void)store.index("b"), std::out_of_range);
}

TEST(DataStore, ValuesStartUnreceivedAndTrackStamp)
{
  DataStore store;
  const auto i = store.add("imu/orientation.w");
  EXPECT_FALSE(store.received(i));
  store.set(i, 1.0, 12.5);
  EXPECT_TRUE(store.received(i));
  EXPECT_DOUBLE_EQ(store.get(i), 1.0);
  EXPECT_DOUBLE_EQ(store.stamp(i), 12.5);
}

TEST(DataStore, CopyValuesFromCopiesValuesStampsAndReceivedFlags)
{
  DataStore a;
  a.add("x");
  a.add("y");
  DataStore b = a;  // same layout
  a.set(0, 1.5, 7.0);
  b.set(1, 9.0, 3.0);
  b.copy_values_from(a);
  EXPECT_DOUBLE_EQ(b.get(0), 1.5);
  EXPECT_DOUBLE_EQ(b.stamp(0), 7.0);
  EXPECT_TRUE(b.received(0));
  EXPECT_FALSE(b.received(1));  // a never received y
}

TEST(DataStore, AllReceivedChecksEveryIndex)
{
  DataStore store;
  const auto a = store.add("a");
  const auto b = store.add("b");
  store.set(a, 0.0, 0.0);
  EXPECT_FALSE(store.all_received({a, b}));
  store.set(b, 0.0, 0.0);
  EXPECT_TRUE(store.all_received({a, b}));
}
