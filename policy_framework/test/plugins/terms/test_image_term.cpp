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

#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "policy_framework/plugins/terms/image_term.hpp"
#include "rclcpp/rclcpp.hpp"

using policy_framework::DataStore;
using policy_framework::ImageTerm;
using policy_framework::ModelInfo;
using policy_framework::PluginContext;
using policy_framework::TensorMap;
using Image = sensor_msgs::msg::Image;
using Floats = std::vector<float>;

namespace
{

class ImageTermTest : public ::testing::Test
{
protected:
  /// Configures the term on a fresh node; returns its size.
  size_t configure(const std::string & encoding, int width, int height, double scale = 0.0)
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({{"cam.topic", topic_}, {"cam.encoding", encoding},
      {"cam.width", width}, {"cam.height", height}, {"cam.scale", scale}});
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_image_term", opts);
    return term_.configure({node_, "cam"}, ModelInfo{}, state_);
  }

  /// Publishes @p msg best effort, as camera drivers do, until the term received it.
  void receive(Image msg)
  {
    msg.header.stamp.sec = 7;
    auto pub_node = std::make_shared<rclcpp::Node>("test_image_term_pub");
    auto pub = pub_node->create_publisher<Image>(topic_, rclcpp::SensorDataQoS());
    for (int i = 0; i < 100 && term_.receive_time() == 0.0; ++i) {
      pub->publish(msg);
      rclcpp::spin_some(node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(term_.receive_time(), 0.0) << "no image received";
  }

  /// @return The term's value for the received image (checks it is ready and its stamp).
  Floats compute(size_t size)
  {
    EXPECT_TRUE(term_.ready(state_));
    Floats out(size, -1.0F);
    EXPECT_DOUBLE_EQ(term_.compute(state_, TensorMap{}, out.data()), 7.0);
    return out;
  }

  static Image image(const std::string & encoding, uint32_t width, uint32_t height, uint32_t step,
    std::vector<uint8_t> data)
  {
    Image msg;
    msg.encoding = encoding;
    msg.width = width;
    msg.height = height;
    msg.step = step;
    msg.data = std::move(data);
    return msg;
  }

  std::string topic_ = "/test_image_term_" + std::to_string(++count_);
  static inline int count_ = 0;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  DataStore state_;
  ImageTerm term_;
};

}  // namespace

TEST_F(ImageTermTest, IsChannelsTimesHeightTimesWidth)
{
  EXPECT_EQ(configure("rgb8", 4, 3), 3U * 3U * 4U);
}

TEST_F(ImageTermTest, RejectsAnUnsupportedEncodingOrSize)
{
  EXPECT_ANY_THROW(configure("yuv422", 4, 3));
}

TEST_F(ImageTermTest, WaitsForTheFirstImage)
{
  configure("mono8", 1, 1);
  EXPECT_FALSE(term_.ready(state_));
}

TEST_F(ImageTermTest, WritesChannelsFirstScaledToOneAndSkipsTheRowPadding)
{
  const auto n = configure("rgb8", 2, 2);
  // 2x2 RGB, 8 bytes per row: 2 pixels and 2 padding bytes.
  receive(image("rgb8", 2, 2, 8, {
    0, 51, 102, 153, 204, 255, 9, 9,
    255, 0, 0, 0, 255, 0, 9, 9}));
  const auto out = compute(n);
  const Floats expected{
    0, 0.6F, 1, 0,   // R plane, row major
    0.2F, 0.8F, 0, 1,  // G
    0.4F, 1, 0, 0};  // B
  ASSERT_EQ(out.size(), expected.size());
  for (size_t i = 0; i < out.size(); ++i) {EXPECT_NEAR(out[i], expected[i], 1e-6) << i;}
}

TEST_F(ImageTermTest, SwapsBgrIntoTheConfiguredRgbOrder)
{
  const auto n = configure("rgb8", 1, 1, 1.0);
  receive(image("bgr8", 1, 1, 3, {1, 2, 3}));
  EXPECT_EQ(compute(n), (Floats{3, 2, 1}));
}

TEST_F(ImageTermTest, ReplacesInvalidDepthWithZeroAndScales)
{
  const auto n = configure("32FC1", 3, 1, 2.0);
  const float px[3] = {1.5F, std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity()};
  std::vector<uint8_t> data(sizeof(px));
  std::memcpy(data.data(), px, sizeof(px));
  receive(image("32FC1", 3, 1, sizeof(px), data));
  EXPECT_EQ(compute(n), (Floats{3, 0, 0}));
}

TEST_F(ImageTermTest, ReadsSixteenBitDepth)
{
  const auto n = configure("16UC1", 1, 1, 0.001);
  const uint16_t mm = 1500;
  std::vector<uint8_t> data(2);
  std::memcpy(data.data(), &mm, 2);
  receive(image("16UC1", 1, 1, 2, data));
  EXPECT_NEAR(compute(n)[0], 1.5F, 1e-6);
}

TEST_F(ImageTermTest, IsNotReadyWithAnImageThatDoesNotMatch)
{
  configure("rgb8", 2, 2);
  receive(image("rgb8", 1, 2, 3, std::vector<uint8_t>(6, 0)));  // wrong width
  EXPECT_FALSE(term_.ready(state_));
}

TEST_F(ImageTermTest, IsNotReadyWithATruncatedImage)
{
  configure("mono8", 2, 2);
  receive(image("mono8", 2, 2, 2, std::vector<uint8_t>(3, 0)));  // 4 bytes expected
  EXPECT_FALSE(term_.ready(state_));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
