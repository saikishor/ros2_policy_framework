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
     damp [label="Damping\nuntil reactivated", fillcolor="#fde8e8"];
     start -> hold [label="activate"];
     hold -> run [label="first commands\nfrom the policy"];
     run -> run [label="policy waits for data:\nkeep the last commands"];
     run -> safe [label="fault\n(on_fault: stop)"];
     run -> damp [label="fault\n(on_fault: damp)"];
     hold -> safe [label="deactivate"];
     run -> safe [label="deactivate"];
     damp -> safe [label="deactivate"];
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

Faults
~~~~~~

A *fault* is any of:

- a stage of the policy fails, including a model that answers a non-finite value (NaN or
  infinity): those values never reach the executors;
- the state interfaces cannot be read (below);
- a safety check is violated (below).

The parameter ``on_fault`` decides what happens:

``stop`` (default)
   Every executor writes its *safe values*: zero effort for ``PdEffortExecutor``, nothing for
   ``ReferenceInterfaceExecutor``. ``update()`` returns an error to the controller manager.

``damp``
   The controller stays active but stops following the policy. On every update every
   executor writes its *damping values*: ``PdEffortExecutor`` writes
   ``effort = -fault_damping * velocity`` per joint (within ``effort_limits``), so a falling
   robot goes down slowly instead of limp. Every joint needs a ``fault_damping`` above zero:
   the controller refuses to configure with ``on_fault: damp`` otherwise. The fault is latched:
   it is cleared only by deactivating and activating the controller again, never by itself.

When the controller is deactivated every executor writes its safe values.

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
   * - An input stops arriving although it was received before (``safety.max_input_age``)
     - The policy keeps stepping on the last values; once the observation of the last step is
       older than the limit, it is a fault.

Safety checks
-------------

The parameters ``safety.*`` add checks on the measured state, all off by default. They run on
every controller update (every node tick) and the first one violated is a fault (in the node:
no more commands are sent until it is reactivated, and the reason is logged once). A value
that was never received is not checked.

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Parameter
     - Fault when
   * - ``safety.max_tilt``, ``safety.imu``
     - the IMU z axis is tilted more than ``max_tilt`` (rad) from the vertical: the robot is
       falling. Reads ``<imu>/orientation.{x,y,z,w}``.
   * - ``safety.max_joint_velocity``
     - the absolute velocity of a commanded joint exceeds it (``<joint>/velocity``).
   * - ``safety.joint_position_limits``, ``safety.position_margin``
     - the position of a commanded joint leaves the limits of the robot description, narrowed
       by the margin. Continuous joints have no limits.
   * - ``safety.max_input_age``
     - the observation of the last policy step (its oldest input) is older than this many
       seconds: a topic stopped, so the policy acts on the past.

*Commanded joints* are those the policies write a ``<joint>/position`` command for. The state
entries the checks read are claimed as state interfaces by the controller, like any input.

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

What the framework does not do
------------------------------

The safety checks detect a state the robot must not be in; they do not prevent it: the
commands are not clamped to the joint limits, and nothing decides when to switch policies.
``PdEffortExecutor`` can clamp efforts (``effort_limits``). The framework is not a safety
system: an emergency stop and the robot's own protections stay in charge.
