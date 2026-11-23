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

#include "policy_framework/plugins/terms/image_term.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

#include "pluginlib/class_list_macros.hpp"
#include "policy_framework/image_term_parameters.hpp"

namespace policy_framework
{

namespace
{

bool host_is_big_endian()
{
  const uint16_t one = 1;
  uint8_t first = 0;
  std::memcpy(&first, &one, 1);
  return first == 0;
}

/// Pixel value @p i of a row as double; memcpy: the row of a message has no alignment.
template<typename T>
double read(const uint8_t * row, size_t i)
{
  T v;
  std::memcpy(&v, row + i * sizeof(T), sizeof(T));
  return static_cast<double>(v);
}

}  // namespace

size_t ImageTerm::configure_term(const PluginContext & ctx, const ModelInfo &)
{
  const auto params = image_term::ParamListener(ctx.node, ctx.name).get_params();
  logger_ = ctx.node->get_logger();
  clock_ = ctx.node->get_clock();
  name_ = ctx.name;
  encoding_ = params.encoding;
  width_ = static_cast<size_t>(params.width);
  height_ = static_cast<size_t>(params.height);
  const bool color = encoding_ == "rgb8" || encoding_ == "bgr8";
  channels_ = color ? 3 : 1;
  pixel_bytes_ = encoding_ == "32FC1" ? 4 : encoding_ == "16UC1" ? 2 : channels_;
  scale_ = params.scale > 0.0 ? params.scale : (pixel_bytes_ == channels_ ? 1.0 / 255.0 : 1.0);
  return channels_ * height_ * width_;
}

bool ImageTerm::matches(const sensor_msgs::msg::Image & msg) const
{
  const bool color_swap = (encoding_ == "rgb8" && msg.encoding == "bgr8") ||
    (encoding_ == "bgr8" && msg.encoding == "rgb8");
  // Multi-byte pixels in the other byte order would need swapping; no driver we know sends them.
  const bool byte_order_ok = pixel_bytes_ == channels_ ||
    static_cast<bool>(msg.is_bigendian) == host_is_big_endian();
  return (msg.encoding == encoding_ || color_swap) && byte_order_ok &&
         msg.width == width_ && msg.height == height_ && msg.step >= width_ * pixel_bytes_ &&
         msg.data.size() >= static_cast<size_t>(msg.step) * height_;
}

bool ImageTerm::ready(const DataStore &)
{
  // compute() reads this very message: a newer one may arrive in between, unchecked.
  checked_ = latest();
  const auto & in = checked_;
  if (!in.msg) {return false;}
  if (matches(*in.msg)) {return true;}
  RCLCPP_ERROR_THROTTLE(
    logger_, *clock_, 2000,
    "[%s] image %ux%u %s (step %u, %zu bytes) is not the configured %zux%zu %s: not used",
    name_.c_str(), in.msg->width, in.msg->height, in.msg->encoding.c_str(), in.msg->step,
    in.msg->data.size(), width_, height_, encoding_.c_str());
  return false;
}

double ImageTerm::compute(const DataStore &, const TensorMap &, float * out)
{
  const auto & in = checked_;
  const auto & msg = *in.msg;
  const bool swap = msg.encoding != encoding_;  // rgb8 <-> bgr8, see matches()
  const size_t plane = height_ * width_;
  for (size_t y = 0; y < height_; ++y) {
    const uint8_t * row = msg.data.data() + y * msg.step;
    for (size_t x = 0; x < width_; ++x) {
      for (size_t c = 0; c < channels_; ++c) {
        double v = 0.0;
        if (pixel_bytes_ == channels_) {
          v = row[x * channels_ + c];
        } else if (pixel_bytes_ == 2) {
          v = read<uint16_t>(row, x);
        } else {
          v = read<float>(row, x);
        }
        // No depth reading (NaN, inf) would poison the whole policy output.
        if (!std::isfinite(v)) {v = 0.0;}
        const size_t channel = swap ? channels_ - 1 - c : c;
        out[channel * plane + y * width_ + x] = static_cast<float>(v * scale_);
      }
    }
  }
  const auto & s = msg.header.stamp;
  return (s.sec == 0 && s.nanosec == 0) ? in.receive_time : s.sec + s.nanosec * 1e-9;
}

}  // namespace policy_framework

PLUGINLIB_EXPORT_CLASS(policy_framework::ImageTerm, policy_framework::ObservationTerm)
