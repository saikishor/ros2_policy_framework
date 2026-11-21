Safety behaviour
================

This page lists what the framework does when something is missing, late or wrong. All of it
applies to the controller; the parts that apply to the node are marked.

From activation to deactivation
-------------------------------

.. graphviz::

   digraph lifecycle {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     start [shape=point];
     hold [label="Hold\nexecutors keep the robot\nwhere it is", fillcolor="#fff6d9"];
     run [label="Run\nexecutors follow\nthe policy's commands", fillcolor="#e8f5e9"];
     safe [label="Safe values\nzero effort", fillcolor="#fde8e8"];
     start -> hold [label="activate"];
     hold -> run [label="first commands\nfrom the policy"];
     run -> run [label="policy waits for data:\nkeep the last commands"];
     run -> safe [label="a stage fails, or\ninterfaces unreadable"];
     hold -> safe [label="deactivate"];
     run -> safe [label="deactivate"];
   }

Hold
~~~~

Between the activation of the controller and the first commands of the policy there is a
gap. It is short when everything is ready (one inference), and it can be long when the
policy is waiting for an input that has not arrived yet. During that gap the executors
*hold* the robot:

- ``PdEffortExecutor`` runs its PD law around the joint positions measured at activation,
  with a zero velocity reference. It uses the gains the policy provides before its first
  step, or else its own ``stiffness`` and ``damping`` parameters. A joint without gains is
  left untouched.
- ``ReferenceInterfaceExecutor`` writes nothing: the chained controller keeps holding its
  own reference.

Run
~~~

Once the policy has produced commands, the executors follow them on every update. If a later
step has to wait for data, the last commands stay applied.

Safe values
~~~~~~~~~~~

When the controller is deactivated, when a stage of the policy fails, or when the state
interfaces cannot be read (below), every executor writes its *safe values*: zero effort for
``PdEffortExecutor``, nothing for ``ReferenceInterfaceExecutor``. In the failure cases
``update()`` also returns an error to the controller manager.

Inputs that are missing or late
-------------------------------

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Situation
     - What happens
   * - An input has never been received
     - The policy step is skipped (*waiting for data*, logged once). Node: nothing is sent.
       Controller: hold, or the last commands.
   * - A velocity command is old (``timeout``)
     - It is replaced by zero. The robot stops walking instead of following a command nobody
       is sending any more. The same applies before the first command arrives, so a policy
       does not wait for one to start.
   * - A motion reference is old or malformed
     - The goal becomes a still stand at the robot's heading.
   * - A state interface cannot be read (controller)
     - This happens when its writer holds it, for example an asynchronous hardware component.
       The previous value is used for up to ``max_failed_reads`` updates in a row. One more
       failure writes the safe values and returns an error.
   * - Inference is slower than ``policy_rate`` (``async_inference``)
     - The policy tick is skipped and a warning is logged. The executors keep running.

The safety entry
----------------

Robots usually have a signal that says whether it is safe to accept commands from a person:
a dead-man switch, a mode selector, a safety controller. The parameter ``safety_entry`` names
the state entry carrying that signal, for example ``rl_safety_controller/safe``. In the
controller it is claimed as a state interface like any other input.

While that entry is 0 (or has not been received), **user commands are ignored and replaced
by neutral ones**. The policy itself keeps running, because a legged robot has to keep
balancing:

.. graphviz::

   digraph gate {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     user [label="User command\n(velocity, motion reference)", shape=plaintext, fillcolor=none];
     gate [label="safety entry > 0 ?", shape=diamond, fillcolor="#fff6d9"];
     real [label="The command"];
     neutral [label="Neutral command\nzero velocity, still stand", fillcolor="#fde8e8"];
     policy [label="Policy\n(always running)", fillcolor="#e8f5e9"];
     user -> gate;
     gate -> real [label="yes"];
     gate -> neutral [label="no"];
     real -> policy;
     neutral -> policy;
   }

The plugins that feed user commands implement this with ``CommandGate``: ``TwistSource``,
``TwistCommandTerm`` and ``MotionGoalTerm``. Every change of the signal is logged once.
Without ``safety_entry`` commands are always accepted.

Safety checks (not run yet)
---------------------------

Checks specific to a robot, such as an actuator reporting an error or a motor running hot,
are ``SafetyCheck`` plugins. Each one registers the state entries it reads in ``configure()``
and says in ``check()`` whether the state is safe, with a reason when it is not. ``check()``
runs in the control loop: it must not allocate while the state is safe.

``StateBoundsCheck`` checks every ``<prefix>/<interface>`` entry against the bounds of its
interface. A non-finite value is out of bounds; an entry never received is not checked:

.. code-block:: yaml

   motors:
     type: policy_framework/StateBoundsCheck
     prefixes: [leg_left_1_joint, leg_left_2_joint]
     interfaces: [temperature, error_code]
     temperature: {max: 85.0}
     error_code: {min: 0.0, max: 0.0}   # any error code but 0 fails
     trigger_cycles: 3                  # consecutive violations before it fails

``JumpCheck`` fails when an entry changes more than ``<interface>.max_delta`` between two
cycles, for example an encoder glitch, or when it is not finite. The first value is never a
jump. Do not use it on values that wrap, such as the position of a continuous joint in
[-pi, pi]:

.. code-block:: yaml

   encoders:
     type: policy_framework/JumpCheck
     prefixes: [leg_left_1_joint, leg_left_2_joint]
     interfaces: [position]
     position: {max_delta: 0.2}

The node and the controller do not load these plugins yet: a failing check has no effect.

Safety monitor (not run yet)
----------------------------

Besides the plugins, ``SafetyMonitor`` has built-in checks of the measured state, each off
until it is given a limit. ``check()`` returns the first one violated; a value that was never
received is not checked.

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Check
     - Violated when
   * - Tilt (``max_tilt``, ``imu``)
     - the IMU z axis is tilted more than ``max_tilt`` (rad) from the vertical: the robot is
       falling. Reads ``<imu>/orientation.{x,y,z,w}``.
   * - Joint velocity (``max_joint_velocity``)
     - the absolute velocity of a commanded joint exceeds it (``<joint>/velocity``).
   * - Joint position (``joint_limits``, ``position_margin``)
     - the position of a commanded joint leaves its limits, narrowed by the margin. The limits
       can be read from the robot description; continuous joints have none.
   * - Input age (``max_input_age``)
     - the observation of the last policy step (its oldest input) is older than this many
       seconds: a topic stopped, so the policy acts on the past.

*Commanded joints* are those the policies write a ``<joint>/position`` command for.

The node and the controller do not run the monitor yet: a violation has no effect.

What the framework does not do
------------------------------

It does not limit joint positions or velocities, detect falls, or decide when to stop a
policy. ``PdEffortExecutor`` can clamp efforts (``effort_limits``). Anything beyond that
belongs to the robot's safety layer.
