ros2_policy_framework
=====================

Run learned policies (for example ONNX policies trained with mjlab) on ROS 2 robots.

The framework answers three questions separately, so that each answer can change without
touching the others:

#. **Where does the data come from?** Topics, ros2_control state interfaces, or a mix.
#. **What does the policy do with it?** Build an observation, run a model, turn its output
   into joint commands.
#. **Where do the commands go?** Topics, or ros2_control command interfaces.

The same policy configuration runs as a standalone ROS 2 node or inside a ros2_control
controller, and several policies can be loaded at once and switched while the robot moves.

These pages explain the concepts; :doc:`features` lists what the framework offers. The ``README.md`` at the root of the repository is the
reference: every parameter, every plugin and the commands to run the Kangaroo example.

.. toctree::
   :maxdepth: 2
   :caption: Concepts

   features
   overview
   data_flow
   hosts
   policies
   safety
   observability
   kinematics
   mjlab
   extending
