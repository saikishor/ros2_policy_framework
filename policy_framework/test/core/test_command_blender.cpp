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

#include "policy_framework/core/command_blender.hpp"

using policy_framework::CommandBlender;
using policy_framework::DataStore;

namespace
{

class CommandBlenderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    position = target.add("j1/position");
    stiffness = target.add("j1/stiffness");
    blender.configure(target);
  }
  double out(size_t i) const {return blender.output().get(i);}

  DataStore target;
  size_t position = 0;
  size_t stiffness = 0;
  CommandBlender blender;
};

}  // namespace

TEST_F(CommandBlenderTest, OutputFollowsTheTargetWithoutARamp)
{
  EXPECT_FALSE(blender.output().received(position));
  target.set(position, 1.0, 4.0);
  blender.update(target, 0.001);
  EXPECT_DOUBLE_EQ(out(position), 1.0);
  EXPECT_DOUBLE_EQ(blender.output().stamp(position), 4.0);
  EXPECT_FALSE(blender.output().received(stiffness));  // the target never received it
  EXPECT_FALSE(blender.blending());
}

TEST_F(CommandBlenderTest, RampsLinearlyFromTheOutputAtStartToTheTarget)
{
  target.set(position, 1.0, 0.0);
  target.set(stiffness, 100.0, 0.0);
  blender.update(target, 0.1);

  blender.start(1.0);
  target.set(position, 3.0, 0.0);  // the new policy's commands
  target.set(stiffness, 50.0, 0.0);
  blender.update(target, 0.25);
  EXPECT_TRUE(blender.blending());
  EXPECT_DOUBLE_EQ(out(position), 1.5);
  EXPECT_DOUBLE_EQ(out(stiffness), 87.5);  // gains are blended too
  blender.update(target, 0.25);
  EXPECT_DOUBLE_EQ(out(position), 2.0);
  // The target may keep moving during the ramp: the output converges to where it is.
  target.set(position, 5.0, 0.0);
  blender.update(target, 0.25);
  EXPECT_DOUBLE_EQ(out(position), 4.0);
  blender.update(target, 0.5);  // past the end
  EXPECT_DOUBLE_EQ(out(position), 5.0);
  EXPECT_FALSE(blender.blending());
}

TEST_F(CommandBlenderTest, AnEntryWithoutAStartTakesTheTargetAtOnceUnlessGivenOne)
{
  blender.start(1.0);
  EXPECT_FALSE(blender.has_start(position));
  blender.set_start(position, 0.0);  // e.g. the measured joint position
  target.set(position, 2.0, 0.0);
  target.set(stiffness, 100.0, 0.0);
  blender.update(target, 0.5);
  EXPECT_DOUBLE_EQ(out(position), 1.0);
  EXPECT_DOUBLE_EQ(out(stiffness), 100.0);
}

TEST_F(CommandBlenderTest, ResetForgetsTheOutputAndTheRamp)
{
  target.set(position, 1.0, 0.0);
  blender.update(target, 0.1);
  blender.start(1.0);
  blender.reset();
  EXPECT_FALSE(blender.output().received(position));
  EXPECT_FALSE(blender.blending());
  target.set(position, 2.0, 0.0);
  blender.update(target, 0.1);
  EXPECT_DOUBLE_EQ(out(position), 2.0);
}
