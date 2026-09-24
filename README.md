# ros2_policy_framework

Framework to run learned policies (e.g. mjlab-trained ONNX policies) on ROS 2 robots, with a clear
split between data acquisition, preprocessing, inference, postprocessing and execution.

## Packages

| Package | Content |
|---------|---------|
| `policy_framework` | Core library, plugin base classes, generic plugins, standalone node and ros2_control controller |
