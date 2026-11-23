#!/usr/bin/env python3
# Copyright 2026 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Generate the tiny ONNX models used by the unit tests (run from this directory)."""

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper


def affine_model():
    """actions = obs @ W + b, echo = time_step; batch dimension left dynamic."""
    w = numpy_helper.from_array(np.array([[1, 0], [0, 2], [1, 1]], dtype=np.float32), "W")
    b = numpy_helper.from_array(np.array([0.5, -0.5], dtype=np.float32), "b")
    graph = helper.make_graph(
        [
            helper.make_node("MatMul", ["obs", "W"], ["mm"]),
            helper.make_node("Add", ["mm", "b"], ["actions"]),
            helper.make_node("Identity", ["time_step"], ["echo"]),
        ],
        "affine",
        [
            helper.make_tensor_value_info("obs", TensorProto.FLOAT, ["batch", 3]),
            helper.make_tensor_value_info("time_step", TensorProto.FLOAT, [1, 1]),
        ],
        [
            helper.make_tensor_value_info("actions", TensorProto.FLOAT, ["batch", 2]),
            helper.make_tensor_value_info("echo", TensorProto.FLOAT, [1, 1]),
        ],
        initializer=[w, b],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 7
    helper.set_model_props(model, {"joint_names": "a,b", "action_scale": "0.5,0.5"})
    onnx.checker.check_model(model)
    onnx.save(model, "affine.onnx")


def sensor_model():
    """mjlab model reading only a 3-float `sensor` term (a plugin): actions = sensor.x."""
    w = numpy_helper.from_array(np.array([[1], [0], [0]], dtype=np.float32), "W")
    graph = helper.make_graph(
        [helper.make_node("MatMul", ["obs", "W"], ["actions"])],
        "sensor",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, 3])],
        [helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 1])],
        initializer=[w],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 7
    helper.set_model_props(
        model, {"joint_names": "a", "default_joint_pos": "0.0", "observation_names": "sensor"})
    onnx.checker.check_model(model)
    onnx.save(model, "sensor.onnx")


if __name__ == "__main__":
    affine_model()
    sensor_model()
