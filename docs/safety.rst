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
     run -> safe [label="a stage fails (or NaN),\nor interfaces unreadable"];
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

A model that answers a non-finite value (NaN or infinity), as a diverged one does, counts as a
failed stage: those values never reach the commands. The node sends nothing for that step.

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
   * - Inference is slower than ``policy_rate``, or its thread is busy for an instant, at a
       policy tick (``async_inference``)
     - The policy tick is skipped and a warning is logged. The executors keep running.

The safety entry
----------------

Robots usually have a signal that says whether it is safe to accept commands from a person:
a dead-man switch, a mode selector, a safety controller. The parameter ``safety_entry`` names
the state entry carrying that signal, for example ``rl_safety_controller/safe``. In the
controller it is claimed as a state interface like any other input.

While that entry is 0 (or has not been received), or while a fault with the
``NEUTRAL_COMMANDS`` reaction is active, **user commands are ignored and replaced by neutral
ones**. The policy itself keeps running, because a legged robot has to keep
balancing:

.. graphviz::

   digraph gate {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     user [label="User command\n(velocity, motion reference)", shape=plaintext, fillcolor=none];
     gate [label="safety entry > 0 (if set)\nand no neutral commands\nreaction ?", shape=diamond, fillcolor="#fff6d9"];
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
Without ``safety_entry`` commands are accepted unless the ``NEUTRAL_COMMANDS`` reaction is
active.

Safety checks
-------------

Checks specific to a robot, such as an actuator reporting an error or a motor running hot,
are ``SafetyCheck`` plugins. Each one registers the state entries it reads in ``configure()``
and returns from ``check()`` a ``Verdict``: whether the state is safe and, when it is not,
optionally the reaction it asks for (see `A reaction chosen while running`_), with a reason.
``check()`` runs in the control loop: it must not allocate while the state is safe. Writing
one is shown in :doc:`extending`.

``StateBoundsCheck`` checks every ``<prefix>/<interface>`` entry against the bounds of its
interface. A non-finite value is out of bounds; an entry never received is not checked. With a
``hysteresis``, an entry that failed passes again only within the bounds narrowed by it:

.. code-block:: yaml

   safety_checks: [motors]
   motors:
     type: policy_framework/StateBoundsCheck
     prefixes: [leg_left_1_joint, leg_left_2_joint]
     interfaces: [temperature, error_code]
     temperature: {max: 85.0, hysteresis: 5.0}   # once failed, passes again below 80
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

They are listed in ``safety_checks``, each with its ``type`` and its reaction ``on_fault``
(default ``error``, see `Fault reactions`_). ``policy_node`` runs them on every tick, before the policy step.
``PolicyController`` loads them and claims what they read, but does not run them yet (it
warns about it).

Safety monitor
--------------

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
   * - Joint position (``joint_limits``, ``position_margin``)
     - the position of a commanded joint leaves its limits, narrowed by the margin. The limits
       can be read from the robot description; continuous joints have none.
   * - Input age (``max_input_age``)
     - the observation of the last policy step (its oldest input) is older than this many
       seconds: a topic stopped, so the policy acts on the past.

*Commanded joints* are those the policies write a ``<joint>/position`` command for.

There is no joint velocity check in the monitor: a ``StateBoundsCheck`` does it, with a
limit per interface and only on the joints you list:

.. code-block:: yaml

   knee_velocity:
     type: policy_framework/StateBoundsCheck
     prefixes: [leg_left_4_joint, leg_right_4_joint]
     interfaces: [velocity]
     velocity: {min: -12.0, max: 12.0}

Its parameters are ``safety.max_tilt`` with ``safety.imu``, ``safety.joint_position_limits``
(the limits of the robot description) with ``safety.position_margin``, and
``safety.max_input_age``; each check has its reaction (``safety.tilt_on_fault``,
``safety.joint_position_on_fault``, ``safety.input_age_on_fault``, default ``error``).
``policy_node`` runs it with the safety checks; ``PolicyController`` does not yet.

Fault reactions
---------------

What to do about a fault depends on what went wrong: after an actuator error in an arm it is
wiser to stop walking and keep balancing, while a leg actuator failing or a joint going far
too fast is better stopped at once. The reactions, from the most control kept to the least:

.. list-table::
   :header-rows: 1
   :widths: 18 41 41

   * - Reaction
     - ``PdEffortExecutor``
     - ``ReferenceInterfaceExecutor``
   * - ``NEUTRAL_COMMANDS``
     - the policy keeps running and the executors keep following it, with neutral user
       commands: zero velocity, a still stand (see `The safety entry`_)
     - same
   * - ``HOLD``
     - PD law around the position at the fault; zero effort for a joint without gains
     - not supported
   * - ``DAMP``
     - ``effort = -fault_damping * velocity`` within ``effort_limits``; needs a
       ``fault_damping`` above zero on every joint
     - not supported
   * - ``SAFE``
     - zero effort
     - nothing written: the chained controller keeps its last reference
   * - ``ERROR``
     - the controller returns an error: the controller manager deactivates it (it writes its
       safe values) and activates its ``fallback_controllers``
     - same

On every cycle the runner runs every check, takes the most severe reaction among the failing
ones, and applies it; only neutral commands recover by themselves:

.. graphviz::

   digraph reactions {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     start [shape=point];
     ok [label="Every check passes\npolicy followed", fillcolor="#e8f5e9"];
     neutral [label="Neutral commands\npolicy followed,\nuser commands neutral", fillcolor="#fff6d9"];
     latched [label="Latched reaction\nhold, damp, safe or error", fillcolor="#fde8e8"];
     start -> ok [label="activate"];
     ok -> neutral [label="a failing check asks\nfor neutral_commands"];
     neutral -> ok [label="it passes again"];
     ok -> latched [label="a failing check (or a\nfailed step) asks for more"];
     neutral -> latched [label="a more severe\nreaction"];
     latched -> latched [label="escalates to a more\nsevere one, never back"];
     latched -> ok [label="deactivate and\nactivate", style=dashed];
   }

The plugin decides *whether*, the configuration decides *what*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A safety check only says whether the state is unsafe, and why. The reaction is set in the
configuration, per instance of a check, next to its ``type`` (``on_fault``, default
``error``). Several instances of the same plugin, each on its own joints or with its own
bounds, give each group its reaction:

.. code-block:: yaml

   safety_checks: [leg_actuators, arm_actuators, motors_warm, motors_hot]
   leg_actuators:            # a main joint fails: unload, the fallback controllers take over
     type: policy_framework/StateBoundsCheck
     prefixes: [leg_left_1_joint, leg_left_2_joint]
     interfaces: [error_code]
     error_code: {min: 0.0, max: 0.0}
     on_fault: error
   arm_actuators:            # an upper body joint fails: stop walking, keep balancing
     type: policy_framework/StateBoundsCheck
     prefixes: [arm_left_1_joint, arm_left_2_joint]
     interfaces: [error_code]
     error_code: {min: 0.0, max: 0.0}
     on_fault: neutral_commands
   motors_warm:              # a graded severity on the same entries: two instances
     type: policy_framework/StateBoundsCheck
     prefixes: [leg_left_1_joint, leg_left_2_joint]
     interfaces: [temperature]
     temperature: {max: 80.0, hysteresis: 10.0}   # passes again below 70
     on_fault: neutral_commands
   motors_hot:
     type: policy_framework/StateBoundsCheck
     prefixes: [leg_left_1_joint, leg_left_2_joint]
     interfaces: [temperature]
     temperature: {max: 95.0}
     on_fault: damp

The behaviour is still dynamic while running:

- **Going**: the checks that fail decide the reaction; the most severe one wins.
- **Recovering**: ``NEUTRAL_COMMANDS`` lifts as soon as no failing check asks for it any more,
  so the robot walks again once its motors cooled down. Give such a check a ``hysteresis`` so
  that a value hovering at a bound does not toggle it. Every other reaction is latched until
  the controller is reactivated.
- **Escalating**: a faulted controller only ever moves to a later reaction, never back. Warm
  motors stop the walking, hot ones damp the robot down, a failing leg actuator returns an
  error.

Why the reaction is set in the configuration rather than chosen by the plugins:

- **It can be audited**: reading the configuration tells what happens on every fault, which
  matters on a robot.
- **It is checked at startup**: every reaction in use is known while configuring, so a
  reaction an executor does not support with its configuration (for example ``DAMP`` without
  a ``fault_damping``) will be refused then, not discovered during a fall.
- **Plugins stay simple and reusable**: they only answer "is this unsafe, and why"; the same
  ``StateBoundsCheck`` serves every joint group with a different reaction.

There is no global default reaction on purpose. With one, a check without its own reaction
would not say what it does, and changing the default would silently change every such check.
Instead each check has its own, ``error`` until set: an unset check does the safest thing,
and the reaction of every check can be read next to it.

A reaction chosen while running
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Sometimes the reaction depends on the failure itself: one actuator in error may only stop the
walking, while two or three at once call for an error. ``SafetyCheck::check()`` returns a
``Verdict``: whether the state is safe and, when it is not, optionally another reaction than
the configured ``on_fault``, among the reactions the check declares once configured
(``SafetyCheck::possible_reactions()``). Both points
above still hold: the declared reactions come from the configuration, so they are known and
checked at startup.

``StateBoundsCheck`` does it with ``escalation``: from ``escalation.count`` entries failing at
once, it asks for ``escalation.on_fault`` instead of its ``on_fault``:

.. code-block:: yaml

   leg_actuators:
     type: policy_framework/StateBoundsCheck
     prefixes: [leg_left_1_joint, leg_left_2_joint, leg_left_3_joint, leg_left_4_joint]
     interfaces: [error_code]
     error_code: {min: 0.0, max: 0.0}
     on_fault: neutral_commands   # one actuator in error: stop walking, keep balancing
     escalation:
       count: 2                   # two or more at once: unload, the fallback controllers take over
       on_fault: error

Escalating still only goes one way: once the controller returned an error, fewer failing
entries do not bring it back. Other logic (which entries fail, for how long, combined with
the kinematics) belongs in your own ``SafetyCheck``, which declares the reactions it may ask
for.

Going to a predefined pose, or any other recovery, is the job of a fallback controller (for
example a joint trajectory controller): use ``ERROR``.

``PolicyRunner::update_safety()`` runs every check on every cycle and applies the most
severe reaction. A failed policy step (including a non-finite model output) is a fault too,
with its own reaction ``on_policy_failure`` (default ``error``; ``hold``, ``damp`` or ``safe``
otherwise, but not ``neutral_commands``: the policy itself failed, it cannot keep running).
Unreadable state interfaces in the controller still return an error.

- ``policy_node`` does not own the hardware: every latched reaction (anything but
  ``neutral_commands``) stops the commands until the node is reactivated; neutral commands keep
  the policy running.
- ``PolicyController`` does not use the reactions yet; the executors implement them
  (``InterfaceExecutor::fault()``).

What the framework does not do
------------------------------

It does not limit joint positions or velocities, detect falls, or decide when to stop a
policy. ``PdEffortExecutor`` can clamp efforts (``effort_limits``). Anything beyond that
belongs to the robot's safety layer.
