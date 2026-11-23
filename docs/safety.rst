Safety behaviour
================

This page lists what the framework does when something is missing, late or wrong. Both hosts
run the same safety checks, input age checks and command gating. They differ in what they can
do about a fault: the controller owns the hardware and applies a reaction through its
executors, while the node only stops sending commands. Where the node differs, it is said.

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
     fault [label="Fault reaction\nhold, damp or safe,\nuntil reactivated", fillcolor="#fde8e8"];
     error [label="Error\ndeactivated by the controller\nmanager, fallback controllers", fillcolor="#fde8e8"];
     start -> hold [label="activate"];
     hold -> run [label="first commands\nfrom the policy"];
     run -> run [label="policy waits for data:\nkeep the last commands"];
     hold -> fault [label="latched fault"];
     hold -> error [label="reaction error, or\ninterfaces unreadable"];
     run -> fault [label="latched fault\n(a check, or a failed step)"];
     fault -> fault [label="escalates"];
     run -> error [label="reaction error, or\ninterfaces unreadable"];
     fault -> error [label="escalates to error"];
     error -> safe;
     hold -> safe [label="deactivate"];
     run -> safe [label="deactivate"];
     fault -> safe [label="deactivate"];
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

When the controller is deactivated, when the state interfaces cannot be read (below), or on
the ``ERROR`` fault reaction, every executor writes its *safe values*: zero effort for
``PdEffortExecutor``, nothing for ``ReferenceInterfaceExecutor``. In the last two cases
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
   * - A velocity command is old (``timeout``, measured on the receive time)
     - It is replaced by zero. The robot stops walking instead of following a command nobody
       is sending any more. The same applies before the first command arrives, so a policy
       does not wait for one to start. The timeout cannot be disabled: nothing else catches
       an operator who stopped sending (commands do not count for the input age below).
   * - A motion reference is old or malformed
     - The goal becomes a still stand at the robot's heading.
   * - A state interface cannot be read (controller)
     - This happens when its writer holds it, for example an asynchronous hardware component.
       The previous value is used for up to ``max_failed_reads`` updates in a row. One more
       failure writes the safe values and returns an error.
   * - Inference is slower than ``policy_rate``, or its thread is busy for an instant, at a
       policy tick (``async_inference``)
     - The policy tick is skipped and a warning is logged. The executors keep running.
   * - A source receives nothing any more (``<source>.max_age``)
     - Once its latest message arrived longer ago than this many seconds, it is a fault, with
       the reaction ``<source>.max_age_on_fault`` (default ``error``). Off by default.
   * - An observation term with its own subscription receives nothing any more
       (``<policy>.terms.<term>.max_age``)
     - After a step of its policy, once its latest message arrived longer ago than this many
       seconds, it is a fault, with the reaction ``max_age_on_fault`` (default ``error``); the
       step's commands are not applied. Off by default.

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

The plugins that feed user commands implement this with ``CommandGate``: ``TwistSource``, and
the terms derived from ``CommandTerm`` (``TwistCommandTerm``), which also
handles their timeout. Every change of the signal is logged once.
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

They are listed in ``safety_checks``, each with its ``type`` and its reaction ``on_fault``
(default ``error``, see `Fault reactions`_). ``policy_node`` runs them on every tick, before
the policy step. ``PolicyController`` runs them on every update, before the policy, and claims
what they read as state interfaces. The framework ships three:

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

``MockSafetyCheck`` fails on request, to see how the robot really reacts to a fault at a chosen
moment (standing, walking, during a jump) before a real one happens. It subscribes to
``topic`` (default ``~/<name>``, ``policy_framework/msg/MockFault``) and fails on every cycle
while the latest message has ``fault: true``, with the message's ``reaction`` (empty: its
``on_fault``), for its ``duration`` in seconds (0: until the next message). A message may
ask only for the reactions listed in ``reactions``, which the host checks against its
executors at configure, like any reaction a check declares; an unknown name asks for
``error``, and the host applies ``error`` for a valid but undeclared one. As for any
fault, every reaction but ``neutral_commands`` stays latched until reactivation, which also
forgets the latest message, so that the robot does not fault again at once:

.. code-block:: yaml

   mock:
     type: policy_framework/MockSafetyCheck
     on_fault: hold
     reactions: [damp, safe, neutral_commands]

.. code-block:: bash

   ros2 topic pub --once /policy_controller/mock policy_framework/msg/MockFault \
     "{fault: true, reaction: damp, reason: 'test while walking'}"
   # neutral_commands recovers once the check passes again: here after 2 s by itself
   ros2 topic pub --once /policy_controller/mock policy_framework/msg/MockFault \
     "{fault: true, reaction: neutral_commands, duration: 2.0}"
   # or until told otherwise
   ros2 topic pub --once /policy_controller/mock policy_framework/msg/MockFault "{fault: false}"

Remove it from the configuration outside of testing: anyone who can publish on its topic can
stop the robot.

Safety monitor
--------------

Besides the plugins, ``SafetyMonitor`` has built-in checks of the measured state, each off
until it is given a limit. The first one violated is reported; a value that was never
received is not checked. The name in the first column is the one
``disabled_safety_checks`` uses (see `Checks per policy`_).

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Check (parameters)
     - Violated when
   * - ``tilt`` (``safety.max_tilt``, ``safety.imu``)
     - the IMU z axis is tilted more than ``max_tilt`` (rad) from the vertical: the robot is
       falling. Reads ``<imu>/orientation.{x,y,z,w}``.
   * - ``joint_position_limits`` (``safety.joint_position_limits``,
       ``safety.position_margin``)
     - the position of a commanded joint leaves the limits of the robot description, narrowed
       by the margin; continuous joints have none.
   * - ``max_input_age`` (``safety.max_input_age``)
     - the observation of the last policy step (its oldest input, user commands excluded) is
       older than this many seconds: a topic stopped, so the policy acts on the past.

*Commanded joints* are those the policies write a ``<joint>/position`` command for.

There is no joint velocity check in the monitor: a ``StateBoundsCheck`` does it, with a
limit per interface and only on the joints you list:

.. code-block:: yaml

   knee_velocity:
     type: policy_framework/StateBoundsCheck
     prefixes: [leg_left_4_joint, leg_right_4_joint]
     interfaces: [velocity]
     velocity: {min: -12.0, max: 12.0}

Each check has its reaction (``safety.tilt_on_fault``,
``safety.joint_position_on_fault``, ``safety.input_age_on_fault``, default ``error``).
Both hosts run it with the safety checks.

Stale inputs: ``max_input_age`` or ``max_age``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Three settings catch inputs that stopped, and they answer different questions:

.. list-table::
   :header-rows: 1
   :widths: 16 28 28 28

   * -
     - ``safety.max_input_age``
     - ``<source>.max_age``
     - ``<policy>.terms.<term>.max_age``
   * - For
     - The whole observation
     - An input source (topic → state entries)
     - An observation term with its own subscription (``TopicTerm``)
   * - Question
     - How old is the observation the policy last acted on?
     - When did this source last receive a message?
     - When did this term last receive a message?
   * - Measured on
     - The stamps of the step's inputs (header stamps, so the publishers' clocks must agree),
       user commands excluded
     - The receive time, on the node clock: whatever clock the publisher stamps with
     - The receive time, like a source
   * - Limit
     - One for the whole observation: it has to allow for the slowest input
     - One per source, e.g. 20 ms for the IMU and 200 ms for a slow estimator
     - One per term
   * - Checked
     - On every cycle
     - On every cycle
     - After each policy step, when its result is taken (the term keeps returning its last
       message, so the step that would act on it is the one stopped)
   * - Also catches
     - A stuck inference thread: no new observation is acted on
     - A source no active policy reads: every source is checked (only the active policy's,
       with a check when switching, is planned in issue #18)
     - Only the terms of the active policy are checked: a term is read by its policy only

Use ``max_age`` for each sensor topic, with a limit that suits its rate, and
``max_input_age`` as a looser check of the whole observation.

The state interfaces of the controller are read on every update, so they are never old (a
sensor repeating its last value is not caught by an age; see ``JumpCheck`` or a check of your
own). Do not set ``max_age`` on a source of user commands (``TwistSource``): an operator who
stops sending is normal, and the source's ``timeout`` already turns that into a neutral
command; a command term (``CommandTerm``) refuses a ``max_age`` for the same reason. The
reaction of a term's ``max_age`` is ``hold``, ``damp``, ``safe`` or ``error``: neutral
commands would keep the policy running, on the stale data.

.. _checks-per-policy:

Checks per policy
-----------------

Not every limit suits every policy: a jump moves the joints much faster than walking, and may
tilt the base further. A policy lists the checks it does not want in
``<policy>.disabled_safety_checks``; those checks are skipped while it is the active policy,
and every other policy keeps them. A new policy keeps every check unless it says otherwise.

.. code-block:: yaml

   policies: [walk, jump]
   jump:
     max_steps: 147
     interruptible: false
     disabled_safety_checks: [joint_velocity, tilt]
   safety:
     max_tilt: 0.6
     imu: imu
   safety_checks: [joint_velocity, encoders]
   joint_velocity:
     type: policy_framework/StateBoundsCheck
     prefixes: [leg_left_4_joint, leg_right_4_joint]
     interfaces: [velocity]
     velocity: {min: -12.0, max: 12.0}

Here ``walk`` runs with every check; ``jump`` runs without the velocity bound and the tilt
check, but ``encoders`` and every other check still apply.

What can be listed
   The names in ``safety_checks``, and the monitor checks ``tilt``, ``joint_position_limits``
   and ``max_input_age``. An unknown name is refused when configuring, so that a typo does not
   leave a check running unnoticed; for the same reason a ``safety_checks`` entry may not be
   named like a monitor check. A source's or a term's ``max_age`` cannot be disabled: a stale
   input is a fault whatever the policy does.

Looser limits instead of none
   Configure the check twice, with the strict and the loose limits, and let each policy
   disable the one that does not suit it:

   .. code-block:: yaml

      safety_checks: [joint_velocity, joint_velocity_jump]
      joint_velocity:
        type: policy_framework/StateBoundsCheck
        prefixes: [leg_left_4_joint, leg_right_4_joint]
        interfaces: [velocity]
        velocity: {min: -12.0, max: 12.0}
      joint_velocity_jump:
        type: policy_framework/StateBoundsCheck
        prefixes: [leg_left_4_joint, leg_right_4_joint]
        interfaces: [velocity]
        velocity: {min: -25.0, max: 25.0}
      walk:
        disabled_safety_checks: [joint_velocity_jump]
      jump:
        disabled_safety_checks: [joint_velocity]

When it applies
   The checks follow the policy whose commands are applied. A switch takes effect at the start
   of the next policy step, which runs after the checks of that cycle, so the new policy's
   exemptions apply from the cycle after its first step, and the checks it disabled come back
   from the cycle after the next policy's first step (for ``jump`` above, after ``max_steps``,
   when the first policy takes over again).

Skipped, then started afresh
   A disabled check is not run at all, rather than run with its result ignored: a check that
   fails throughout a jump would build its reason on every cycle of the control loop. When it
   applies again it is reset first (``SafetyCheck::reset()``), so that it does not compare with
   values from before the policy that disabled it: a ``JumpCheck`` would otherwise see the
   whole jump as one glitch. A ``StateBoundsCheck`` starts counting its ``trigger_cycles``
   again, so a value still out of bounds fails a few cycles later.

Faults stay latched
   Switching policy never clears a fault: a reaction latched while one policy ran stays latched
   under the next one, until reactivation.

.. warning::

   When ``jump`` hands back to ``walk``, the joints may still move fast from the landing, and
   ``walk``'s velocity bound applies again at once (after its ``trigger_cycles``). Give that
   check enough ``trigger_cycles`` or ``hysteresis`` for a landing, or bound the velocity
   loosely enough for it. There is no grace period after a switch yet.

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
severe reaction; ``PolicyRunner::check_input_ages()`` checks the ``max_age`` of the terms
after each successful step, before its commands are applied. A failed policy step is a fault
too: a stage that fails, including a model that answers a non-finite value (NaN or infinity)
as a diverged one does, whose values therefore never reach the commands. Its reaction is
``on_policy_failure`` (default ``error``; ``hold``, ``damp`` or ``safe``
otherwise, but not ``neutral_commands``: the policy itself failed, it cannot keep running).
Unreadable state interfaces in the controller still return an error.

- ``policy_node`` does not own the hardware: every latched reaction (anything but
  ``neutral_commands``) stops the commands until the node is reactivated; neutral commands keep
  the policy running.
- ``PolicyController`` refuses, while configuring, a reaction of the configuration that one
  of its executors cannot apply. While a reaction is latched it neither steps nor follows the
  policy: on every update its executors write the reaction (``InterfaceExecutor::fault()``),
  or, for ``error``, their safe values, and ``update()`` returns an error: the controller
  manager deactivates the controller and activates its ``fallback_controllers``. With
  ``async_inference`` a step still running is ignored.

What the framework does not do
------------------------------

The checks above detect a fault and the reactions stop the policy; nothing corrects the
commands instead:

- Commands are never clamped to joint position or velocity limits: leaving a limit is a
  fault. Only ``PdEffortExecutor`` clamps its efforts (``effort_limits``).
- There is no self-collision check.
- There is no recovery motion, such as going to a predefined pose: that is the job of a
  fallback controller, reached with ``ERROR``.
- There is no grace period after a policy switch (see `Checks per policy`_).

Anything beyond that belongs to the robot's safety layer.
