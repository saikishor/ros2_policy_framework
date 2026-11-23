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

#ifndef POLICY_FRAMEWORK__PLUGINS__TERMS__IMAGE_TERM_HPP_
#define POLICY_FRAMEWORK__PLUGINS__TERMS__IMAGE_TERM_HPP_

#include <string>

#include "policy_framework/interfaces/observation_term.hpp"
#include "sensor_msgs/msg/image.hpp"

/// @file image_term.hpp
/// @brief Observation term of a camera image (color or depth).

namespace policy_framework
{

/**
 * @brief The latest sensor_msgs/Image of a topic as float32, channels first (`[C, H, W]`, the
 *        layout of an `image [1, C, H, W]` model input), each value `pixel * scale`.
 *
 * Meant for a model input of its own (MjlabPreprocessor `input_terms`), not the flat
 * observation. The image is used as it is: publish it at the model's resolution (e.g. with
 * image_proc's resize and crop), and compressed images decompressed (image_transport
 * republish). The subscription is best effort (sensor data QoS), as camera drivers publish.
 *
 * Parameters (under the term's name):
 * - `topic` (default `image`)
 * - `encoding`: what the model expects: `rgb8`, `bgr8`, `mono8` (color), `16UC1`, `32FC1`
 *   (depth). The image must have that encoding, except that `rgb8` and `bgr8` are converted
 *   into each other.
 * - `width`, `height`: pixels, the model's.
 * - `scale` (default 0: 1/255 for 8-bit encodings, 1 otherwise; e.g. 0.001 for 16UC1 depth in
 *   millimeters to meters).
 * - `max_age`, `max_age_on_fault`: see TopicTerm.
 *
 * Not ready() before the first image, nor while the latest one does not match (size, encoding,
 * truncated data): the policy does not step on a wrong image, and an error is logged
 * (throttled). Non-finite depth values (NaN, inf: no reading) are written as 0.
 *
 * The stamp is the header stamp (the receive time when it is empty). compute() converts the
 * whole image in the cycle: run it in policy_node or with `async_inference` in the controller.
 */
class ImageTerm : public TopicTerm<sensor_msgs::msg::Image>
{
public:
  ImageTerm() : TopicTerm("image") {}

  bool ready(const DataStore & state) override;
  double compute(const DataStore & state, const TensorMap & last_outputs, float * out) override;

protected:
  /// @throws std::runtime_error on an unsupported encoding or a size of 0.
  size_t configure_term(const PluginContext & ctx, const ModelInfo & model) override;
  rclcpp::QoS qos() const override {return rclcpp::SensorDataQoS();}

private:
  /// @return true when @p msg can be read as the configured image.
  bool matches(const sensor_msgs::msg::Image & msg) const;

  rclcpp::Logger logger_ = rclcpp::get_logger("ImageTerm");
  rclcpp::Clock::SharedPtr clock_;  ///< For the throttled log.
  std::string name_;
  std::string encoding_;
  size_t width_ = 0, height_ = 0, channels_ = 0, pixel_bytes_ = 0;
  double scale_ = 1.0;
  /// The message ready() checked, which compute() reads.
  // ponytail: holds the previous image until the next ready(), which then frees it in the step
  // thread (once per frame); hand it back to a non-RT thread if that ever shows in the timings.
  Received checked_;
};

}  // namespace policy_framework

#endif  // POLICY_FRAMEWORK__PLUGINS__TERMS__IMAGE_TERM_HPP_
