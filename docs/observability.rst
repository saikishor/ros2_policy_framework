Recording and replay
====================

A policy that misbehaves on the robot is hard to debug from joint states alone: what matters
is what the *model* saw and what it answered. The framework can publish exactly that on every
step, record it, and run it through a model again offline.

.. graphviz::

   digraph observability {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     host [label="Node or controller\npublish_steps: true"];
     topic [label="~/policy_step\nPolicyStep", shape=plaintext, fillcolor=none];
     bag [label="Bag (MCAP)", shape=cylinder, fillcolor="#fff6d9"];
     replay [label="policy_replay\nany model", fillcolor="#e8f5e9"];
     report [label="Error per output,\nworst step", shape=plaintext, fillcolor=none];
     host -> topic [label="every step"];
     topic -> bag [label="ros2 bag record"];
     bag -> replay;
     replay -> report;
   }

Publishing every step
---------------------

With ``publish_steps: true`` the host publishes a ``policy_framework/msg/PolicyStep`` on
``~/policy_step`` after every policy step, ok or not:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Field
     - Content
   * - ``policy``, ``step``, ``status``
     - The active policy, its steps since activation, and whether the step wrote commands
       (``STATUS_OK``), waited for an input or failed
   * - ``step_time_us``
     - How long the step took (preprocessor, inference, postprocessor)
   * - ``observation_stamp``, ``input_names``, ``input_stamps``
     - When each input of the observation was produced (see :doc:`data_flow`), so the age of
       every term at the time of the step is known
   * - ``inputs``, ``outputs``
     - Every model input and output tensor, exactly as the model got and gave them
   * - ``command_names``, ``commands``
     - Every command entry after the step

The message is built in the thread that ran the step (the inference thread with
``async_inference``) and handed to a real-time publisher: the control loop never waits for it,
and a message is dropped when the previous one is still being sent. It allocates only when the
layout changes (first step, switch to a policy with other tensor sizes). It is meant for
debugging and data collection; leave it off when nobody listens.

Recording
---------

Any ``ros2 bag record`` works. The Kangaroo launch file does it for you:

.. code-block:: bash

   ros2 launch policy_framework_mjlab kangaroo_mjlab.launch.py record:=/tmp/walk_test

which turns ``publish_steps`` on and records the steps with ``/cmd_vel`` and
``/joint_states`` in an MCAP bag. Tools such as PlotJuggler or Foxglove open it directly, and
plot any value of the observation or the actions over time.

Replaying through a model
-------------------------

``policy_replay`` reads the steps of a bag, runs their recorded inputs through a model, and
compares the outputs with the recorded ones:

.. code-block:: bash

   ros2 run policy_framework policy_replay --ros-args \
     -p bag:=/tmp/walk_test -p model.model_path:=/path/to/policy.onnx

prints, here for a short recording of the test model of the package:

.. code-block:: text

   Replayed 271 steps (28 skipped: model not run, or another policy)
   output                        max error     mean error  worst step
   actions                               0              0  walk #1
   echo                                  0              0  walk #1

Steps whose model did not run (waiting for data, failed) are skipped; ``policy`` replays one
policy only. The exit code is 0 when every output stays within ``tolerance``, 1 when not, and 2
on an error, so it can run in a test or a CI job. Uses:

- **Check an export.** A model exported again, converted (e.g. to another opset or runtime),
  or quantised must answer what the deployed one answered, on real robot inputs rather than on
  inputs from simulation.
- **Compare policy versions.** Run a new version on the inputs the old one saw on the robot,
  and see where and how much their actions differ before deploying it.
- **Reproduce a failure.** The step and policy of the largest difference point to the moment
  to look at in the bag.

The model is any ``Inference`` plugin (``inference``, default ``policy_framework/OnnxInference``)
with its parameters under ``model.``. Only the model is replayed: the observation is taken as
recorded, so the preprocessor is not checked by a replay.
