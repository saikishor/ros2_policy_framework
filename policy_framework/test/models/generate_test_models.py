#!/usr/bin/env python3
# Copyright 2026 PAL Robotics S.L.
# SPDX-License-Identifier: Apache-2.0
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


if __name__ == "__main__":
    affine_model()
