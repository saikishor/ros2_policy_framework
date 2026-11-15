Data flow
=========

This page follows the data through one policy step and explains the two data types every
piece exchanges: the *stores* and the *tensors*.

The stores
----------

A store (``DataStore``) is a flat table. Each row, called an *entry*, has:

- a **name**, for example ``leg_left_1_joint/position``;
- a **value**, one floating point number;
- a **stamp**, the time the value was produced, in seconds;
- a **received** flag, false until the entry is written for the first time.

There are two stores: the **state store** for inputs and the **command store** for outputs.

Names follow the ros2_control convention ``<prefix>/<interface>``:

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Entry
     - What it is
   * - ``leg_left_1_joint/position``
     - A joint position (state), or a joint position reference (command)
   * - ``torso_imu_sensor/orientation.x``
     - One component of the IMU orientation quaternion
   * - ``cmd_vel/linear.x``
     - The forward velocity the user asks for
   * - ``leg_left_1_joint/stiffness``
     - The proportional gain a policy wants for that joint (command)

Using these names has a practical consequence. A topic source writes the same entry names
that the equivalent state interfaces have. So whether a joint position comes from the
``/joint_states`` topic or from a state interface is a matter of configuration only: the
policy reads ``leg_left_1_joint/position`` in both cases.

Entries are created once, while the plugins configure themselves: every plugin *registers*
the entries it reads or writes and keeps their index. During the control cycle the plugins
only use those indices. Nothing is looked up by name and nothing is allocated, which keeps
the cycle fast and predictable.

The tensors
-----------

Between the three stages of a policy the data is no longer named numbers but *tensors*: the
blocks of ``float32`` values the model takes and returns, keyed by the model's input and
output names (``TensorMap``). A typical policy has one input tensor ``obs`` and one output
tensor ``actions``.

One step
--------

.. graphviz::

   digraph step {
     rankdir=TB;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];

     sensors [label="Sensors and user commands", shape=plaintext, fillcolor=none];
     read [label="1. Read inputs\ntopic sources and state interfaces"];
     state [label="State store", shape=cylinder, fillcolor="#fff6d9"];
     pre [label="2. Preprocessor\nobservation terms + history"];
     inf [label="3. Inference\nmodel(obs) = actions"];
     post [label="4. Postprocessor\nscale, offset, gains"];
     commands [label="Command store", shape=cylinder, fillcolor="#fff6d9"];
     exec [label="5. Executors\npublish or write interfaces"];
     robot [label="Robot", shape=plaintext, fillcolor=none];

     sensors -> read -> state -> pre;
     pre -> inf [label="input tensors"];
     inf -> post [label="output tensors"];
     post -> commands -> exec -> robot;
     inf -> pre [label="previous outputs\n(next step)", style=dashed, constraint=false];
   }

#. **Read inputs.** Topic sources copy their latest message into the state store. In the
   controller, the state interfaces are copied too.
#. **Preprocessor.** Reads the state store and fills the model's input tensors. This is where
   the observation is built: each observation term computes its current value, and the
   history of past values is kept. If an input has never been received, the preprocessor
   says so and the step is skipped: the policy *waits for data*.
#. **Inference.** Runs the model. It is the only stage that depends on the model format;
   the framework ships ``OnnxInference`` (ONNX Runtime, on the CPU).
#. **Postprocessor.** Reads the output tensors and writes named commands: typically a position
   reference per joint, computed as ``scale * action + offset``.
#. **Executors.** Send the command store to the robot.

The dashed arrow is the feedback many policies need: the preprocessor also receives the
outputs of the *previous* step of the same policy. That is how the "last action" observation
term works, and how a motion policy feeds its own predicted reference back to itself.

Time stamps
-----------

Every entry carries the time its data was produced. The stamps travel with the data:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Data
     - Its stamp
   * - Entry written by a topic source
     - The message header stamp, or the time it was received when the message has none
   * - Entry read from a state interface
     - The time of the update that read it
   * - Observation of a step
     - The oldest stamp among its inputs (inputs fed back from the previous outputs do not
       count)
   * - Command entry
     - The stamp of the observation it was computed from

This makes two questions answerable at any time: *how old was the data the policy just
acted on?* and *how long did it take from sensor to command?* The controller publishes the
first as ``observation_age_ms`` and the duration of each step as ``policy_step_time_us``
through ros2_control's introspection.
