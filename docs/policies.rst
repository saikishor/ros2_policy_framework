Policies and switching
======================

Several policies can be loaded at once, for example one that walks and one that plays a jump
motion. Exactly one is *active* at any time. The others stay loaded and ready, so switching
never loads a model or allocates memory.

.. code-block:: yaml

   policies: [loco, imitation]     # the first one is the default
   loco:
     preprocessor: policy_framework_mjlab/MjlabPreprocessor
     postprocessor: policy_framework_mjlab/MjlabPostprocessor
     model_path: /path/to/loco
   imitation:
     preprocessor: policy_framework_mjlab/MjlabPreprocessor
     postprocessor: policy_framework_mjlab/MjlabPostprocessor
     model_path: /path/to/jump
     max_steps: 147          # the length of the motion; then back to 'loco'
     interruptible: false    # cannot be switched away from while it plays

Switching
---------

A switch is requested with the ``~/switch_policy`` service of the host, from any thread. It
is applied at the beginning of the next step, so one step never mixes two policies.

.. graphviz::

   digraph switching {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     start [shape=point];
     loco [label="loco\n(default policy)"];
     imitation [label="imitation\nmax_steps: 147\ninterruptible: false"];
     start -> loco [label="host activated"];
     loco -> imitation [label="switch_policy(imitation)"];
     imitation -> loco [label="after 147 steps"];
     imitation -> imitation [label="switch_policy(...)\nrejected"];
     loco -> loco [label="switch_policy(loco)\nrestarts it"];
   }

Three per-policy parameters shape this:

``max_steps``
   After this many steps the pipeline returns to the default (first) policy by itself. Use it
   for a motion of a known length.

``interruptible``
   When false, switch requests are rejected while the policy is active. Use it for a motion
   that must not be cut short, such as a jump.

``transition_time``
   Seconds over which the commands ramp to this policy's commands when it takes over
   (default 0: at once). See below.

A fourth one, ``disabled_safety_checks``, skips some safety checks while the policy is active,
for instance a joint velocity bound during a jump; see :ref:`checks-per-policy`.

Transitions
-----------

Two policies rarely agree on where the joints should be at the moment of a switch, and a
policy's first commands after activation rarely match where the robot stands. With
``transition_time`` the executors do not jump: what they read moves linearly, in that time,
from the commands they had to the new policy's commands.

.. graphviz::

   digraph transition {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     policy [label="Active policy\n(commands)"];
     blend [label="Command blender\nstart → policy commands\nover transition_time", fillcolor="#fff6d9"];
     execs [label="Executors"];
     policy -> blend -> execs;
   }

- The ramp starts from the commands the executors had, and after an activation from the
  measured joint positions (the hold).
- Every command is blended: positions, velocities and the gains of ``PdEffortExecutor``.
- The policy keeps running during the ramp; the ramp ends at its latest commands, wherever
  they moved meanwhile.
- In the controller the ramp advances on every update, so it is smooth even at a 50 Hz policy
  rate. It also works with ``async_inference``: it starts when the new policy's first commands
  are collected.
- A fault reaction (``hold``, ``damp``, ``safe``, ``error``) takes over at once, without a
  ramp; the next activation starts a new ramp from the measured joint positions.
  ``neutral_commands`` keeps the policy running, so its ramp goes on.

A model-free policy
-------------------

A policy does not need a model. ``NullInference``, ``NullPreprocessor`` and
``FixedPosePostprocessor`` make one that commands a fixed pose. With a ``transition_time`` it
is the stand-up or stand-by state other frameworks implement as a state machine:

.. code-block:: yaml

   policies: [stand, walk]         # stand first: the robot stands up on activation
   stand:
     inference: policy_framework/NullInference
     preprocessor: policy_framework/NullPreprocessor
     postprocessor: policy_framework/FixedPosePostprocessor
     joints: [leg_left_1_joint, leg_left_2_joint]
     positions: [0.0, -0.3]
     stiffness: [150.0, 150.0]     # optional, for PdEffortExecutor
     damping: [5.0, 5.0]
     transition_time: 2.0          # from where the robot lies to the pose in 2 s
   walk:
     preprocessor: policy_framework_mjlab/MjlabPreprocessor
     postprocessor: policy_framework_mjlab/MjlabPostprocessor
     model_path: /path/to/walk
     transition_time: 0.5

``switch_policy`` then moves between standing and walking, each way in its own time.

What a policy remembers
-----------------------

A policy has memory: the outputs of its previous step, and the history of its observation
terms. What happens to that memory at each event matters, because a model reacts badly to a
past it never saw in training.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Event
     - Effect on the policy's memory
   * - The host is activated
     - Every policy is **cleared**: previous outputs set to zero, history forgotten. The first
       step after a clear fills the whole history with the current values, as training does
       after an environment reset.
   * - The policy becomes active
     - It is **reset**: per-activation state starts over (the step counter, the alignment of
       a motion). Its history is kept.
   * - Another policy is active
     - It **observes** every step: its history keeps following the robot, without running the
       model.

The third row is the important one. While the jump plays, the walking policy keeps recording
what the robot does. When the jump ends, the walking policy resumes with a history of the
last moments of the jump, which is the real recent past, instead of a history frozen from
before the jump.

.. graphviz::

   digraph memory {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     state [label="State store", shape=cylinder, fillcolor="#fff6d9"];
     subgraph cluster_active {
       label="Active policy"; style="rounded,dashed"; fontname="Helvetica";
       a_pre [label="compute()\nhistory + observation"];
       a_inf [label="Inference"];
       a_post [label="Postprocessor"];
       a_pre -> a_inf -> a_post;
     }
     subgraph cluster_inactive {
       label="Every other policy"; style="rounded,dashed"; fontname="Helvetica";
       i_pre [label="observe()\nhistory only", fillcolor="#f3e9fb"];
     }
     commands [label="Command store", shape=cylinder, fillcolor="#fff6d9"];
     state -> a_pre;
     state -> i_pre;
     a_post -> commands;
   }

Gains follow the policy
-----------------------

Different policies are usually trained with different joint gains. Each mjlab policy carries
its gains in the model file, and they are applied when the policy becomes active. See
:doc:`mjlab` for the two ways they reach the joint loop.
