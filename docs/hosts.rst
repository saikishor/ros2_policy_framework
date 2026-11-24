The two hosts
=============

A *host* is the program that runs the cycle. There are two, and they share everything except
how data enters and leaves. Both are built on the same ``PolicyRunner``, which loads the
input sources, the policies and the safety checks (and the node's executors; the controller
loads its interface executors itself) and owns the stores, so a policy is configured the same
way for either.

Standalone node
---------------

``policy_node`` is an ordinary ROS 2 lifecycle node. It does not need ros2_control.

.. graphviz::

   digraph node_host {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     topics [label="Topics\n/joint_states, /imu, /cmd_vel", shape=plaintext, fillcolor=none];
     subgraph cluster_node {
       label="policy_node (timer at update_rate)"; style="rounded,dashed"; fontname="Helvetica";
       sources [label="Input sources"];
       kin [label="Kinematics\n(if enabled)"];
       safety [label="Safety checks", fillcolor="#fff6d9"];
       policy [label="Active policy\n(one step per tick)"];
       ages [label="Term input ages", fillcolor="#fff6d9"];
       execs [label="Topic executors"];
       stop [label="No commands\nuntil reactivated", fillcolor="#fde8e8"];
       wait [label="Nothing sent\nthis tick", fillcolor="#f2f2f2"];
       sources -> kin -> safety -> policy -> ages -> execs;
       policy -> wait [label="waiting for data", style=dashed];
       safety -> stop [label="latched fault"];
       policy -> stop [label="step fails"];
       ages -> stop [label="stale input"];
     }
     joint_ctrl [label="Joint controller\n(runs elsewhere, at its own rate)", fillcolor="#e8f5e9"];
     topics -> sources;
     execs -> joint_ctrl [label="references\n(topic)"];
   }

- Every input must come from a topic. Configuration fails, listing the missing entries, if a
  policy needs something no input source provides.
- A timer runs one policy step per tick, at ``update_rate`` (the policy rate, typically
  50 Hz).
- Commands leave as topics. The node sends *references*; something else closes the joint
  loop at a high rate: a ros2_control ``pid_controller``, or the robot's own driver.
- The safety checks run on every tick, before the step, and the ``max_age`` of the terms
  after it. The node does not own the hardware: a fault with a latched reaction, a failed
  step or a stale input stops the commands until the node is reactivated; neutral commands
  keep the policy running (:doc:`safety`).

Use it when the robot has no ros2_control, or when the policy should live outside the
control process.

ros2_control controller
-----------------------

``PolicyController`` is a ros2_control controller. It runs inside the controller manager,
in the control loop.

.. graphviz::

   digraph controller_host {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     hw_in [label="State interfaces\n(joints, IMU)", shape=plaintext, fillcolor=none];
     topics [label="Topics\n(/cmd_vel ...)", shape=plaintext, fillcolor=none];
     subgraph cluster_ctrl {
       label="PolicyController (update at update_rate)"; style="rounded,dashed"; fontname="Helvetica";
       read [label="Read state\ninterfaces"];
       sources [label="Input sources\n(policy ticks only)"];
       kin [label="Kinematics\n(every update, if enabled)"];
       safety [label="Safety checks\n(every update)", fillcolor="#fff6d9"];
       policy [label="Active policy\n(every N updates)"];
       ages [label="Term input ages\n(after each step)", fillcolor="#fff6d9"];
       execs [label="Interface executors\n(every update)"];
       fault [label="Fault reaction\nexecutors hold, damp or\ntheir safe values; or an error", fillcolor="#fde8e8"];
       hold [label="Until the first step's\ncommands: executors hold", fillcolor="#f2f2f2"];
       read -> kin;
       sources -> kin;
       kin -> safety;
       safety -> policy;
       policy -> ages -> execs;
       safety -> fault [label="latched fault"];
       policy -> fault [label="step fails"];
       ages -> fault [label="stale input"];
       read -> execs [label="joint feedback", style=dashed];
       read -> fault [label="unreadable for more\nthan max_failed_reads:\nsafe values + error"];
       hold -> execs [label="after activation", style=dashed];
     }
     hw_out [label="Command interfaces\n(joint efforts)", shape=plaintext, fillcolor=none];
     hw_in -> read;
     topics -> sources;
     execs -> hw_out;
   }

- **Inputs.** Every entry that no input source provides is claimed as the state interface of
  the same name. Inputs can therefore all be state interfaces, all be topics, or a mix (for
  example joints and IMU from interfaces, the velocity command from a topic).
- **Outputs.** *Interface executors* turn the commands into command interface values. They
  can compute: ``PdEffortExecutor`` runs a joint PD law and writes efforts, so no other
  controller is needed. ``ReferenceInterfaceExecutor`` only copies references into the
  interfaces of a chained controller.
- **Two rates in one loop.** The state interfaces, the kinematics, the safety checks and the
  executors run on every update; the policy and the topic sources only every
  ``update_rate / policy_rate`` updates.
- **Hold until ready.** From activation until the first step returns commands, the
  executors ``hold()`` the robot instead of following the command store, which may still hold
  the previous activation's references.
- **Safety.** The safety checks run on every update, before the policy. While a reaction is
  latched the policy is neither stepped nor followed: the executors write their fault
  reaction on every update (``hold`` and ``damp`` only where the executor supports them; the
  safe values are zero effort for ``PdEffortExecutor``, unchanged interfaces for
  ``ReferenceInterfaceExecutor``), or the controller returns an error so that the
  controller manager deactivates it and activates its fallback controllers
  (:doc:`safety`). State interfaces that cannot be read for more than ``max_failed_reads``
  updates in a row also write the safe values and return an error.
- **Configuration errors.** A topic executor (e.g. ``MultiDofCommandExecutor``) is refused,
  ``policy_rate`` must divide ``update_rate``, and ``async_inference`` with ``is_async`` is
  refused.

Two rates
---------

A policy is trained at a low rate, typically 50 Hz. A joint PD loop wants a high rate,
typically 1 kHz. The controller runs both in the same loop::

   update_rate: 1000      policy_rate: 50      ->  one policy step every 20 updates

   update     1    2    3   ...   20   21   22   ...   40   41
   policy     S                        S                    S
   executors  x    x    x   ...   x    x    x    ...   x    x

Between two policy steps the position references stay the same, while the PD law keeps
using the newest joint positions and velocities on every update.

Where inference runs
--------------------

Running a neural network takes time, and that time varies. Three arrangements are possible.

.. list-table::
   :header-rows: 1
   :widths: 24 38 38

   * - Setting
     - What happens
     - Use it for
   * - default (synchronous)
     - The policy step runs inside ``update()``. The control loop waits for it.
     - Small models, or a controller whose ``update_rate`` is the policy rate.
   * - ``async_inference: true``
     - Only the policy step runs in its own real-time thread. ``update()`` and the executors
       stay in the control loop and never wait.
     - A PD law at a high rate with a policy at a low rate. Recommended in that case.
   * - ``is_async: true``
     - The standard ros2_control option: the whole ``update()`` runs in another thread,
       executors included. Cannot be combined with ``async_inference``.
     - A reference executor feeding a chained controller. Not for a PD law.

With ``async_inference`` a policy step is spread over several updates:

.. graphviz::

   digraph async_inference {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     subgraph cluster_rt {
       label="Control loop (every update)"; style="rounded,dashed"; fontname="Helvetica";
       tick [label="Policy tick:\ncopy the state,\ntrigger a step"];
       between [label="Following updates:\nexecutors keep using\nthe previous commands"];
       collect [label="First update after\nthe step finished\n(before that update's tick):\ncheck term input ages,\ntake its commands"];
       fault [label="Latched fault:\nno tick, no collect;\nexecutors write\ntheir fault reaction", fillcolor="#fde8e8"];
       tick -> between -> collect;
       between -> fault [style=dashed];
       collect -> fault [label="step failed or\nstale input", style=dashed];
     }
     subgraph cluster_thread {
       label="Inference thread"; style="rounded,dashed"; fontname="Helvetica";
       step [label="Preprocessor,\ninference,\npostprocessor", fillcolor="#e8f5e9"];
     }
     tick -> step [label="state snapshot"];
     step -> collect [label="commands"];
   }

A fault does not wait for the inference thread either: from the update in which it is
latched, the executors write its reaction, and a step still running is ignored. A step that
failed, or whose term inputs are stale, is a fault in the update that collects it, and its
commands are not taken. With ``policy_rate`` 0 there is a tick on every update, so a step's
result is collected at the earliest in the next update.

The step works on a *snapshot* of the state store taken at the policy tick, so it sees one
consistent instant even though the control loop keeps updating the live store. If a policy
tick finds the previous step still running, that tick is skipped and a warning is logged:
the model is slower than ``policy_rate``. A tick can also be skipped, rarely, when the thread
happens to hold its lock at that instant (the control loop never waits for it); the next tick
steps again.

Choosing
--------

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Situation
     - Host
   * - The robot runs ros2_control and exposes joint effort interfaces
     - Controller with ``PdEffortExecutor`` and ``async_inference``
   * - The robot runs ros2_control and already has a joint impedance controller
     - Controller with ``ReferenceInterfaceExecutor``, or the node publishing to it
   * - The robot has no ros2_control
     - Node; its driver must close the joint loop
