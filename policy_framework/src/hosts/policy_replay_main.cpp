// Copyright 2026 PAL Robotics S.L.
// SPDX-License-Identifier: Apache-2.0

// Runs the policy steps recorded in a bag through a model and compares its outputs with the
// recorded ones, e.g.
//
//   ros2 run policy_framework policy_replay --ros-args -p bag:=my_bag -p model.model_path:=p.onnx
//
// Exit code: 0 when every output is within `tolerance`, 1 when not, 2 on an error.

#include <cstdio>
#include <exception>
#include <memory>

#include "pluginlib/class_loader.hpp"
#include "policy_framework/plugin_interfaces.hpp"
#include "policy_framework/hosts/policy_replay.hpp"
#include "policy_framework/policy_replay_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("policy_replay");
    pluginlib::ClassLoader<policy_framework::Inference> loader(
      "policy_framework", "policy_framework::Inference");
    try {
      const auto params = policy_replay::ParamListener(node).get_params();
      auto inference = loader.createSharedInstance(params.inference);
      // The model's parameters live under `model.`, e.g. model.model_path.
      inference->configure({node, "model"});
      const auto summary =
        policy_framework::replay_bag(params.bag, params.topic, inference, params.policy);
      std::printf(
        "Replayed %zu steps (%zu skipped: model not run, or another policy)\n",
        summary.replayed, summary.skipped);
      std::printf("%-24s %14s %14s  %s\n", "output", "max error", "mean error", "worst step");
      for (const auto & [name, e] : summary.outputs) {
        const bool ok = e.max_error <= params.tolerance;
        std::printf(
          "%-24s %14.3g %14.3g  %s #%llu%s\n", name.c_str(), e.max_error, e.mean_error,
          e.worst_policy.c_str(), static_cast<unsigned long long>(e.worst_step),
          ok ? "" : "  > tolerance");
        if (!ok) {code = 1;}
      }
      if (summary.replayed == 0) {
        std::fprintf(stderr, "No step was replayed\n");
        code = 2;
      }
    } catch (const std::exception & e) {
      std::fprintf(stderr, "policy_replay: %s\n", e.what());
      code = 2;
    }
  }
  rclcpp::shutdown();
  return code;
}
