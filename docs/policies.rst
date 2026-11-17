Policies and switching
======================

Several policies can be loaded at once, for example one that walks and one that plays a jump
motion. Exactly one is *active* at any time (one per group, see
:ref:`policy-groups`). The others stay loaded and ready, so switching never loads a model or
allocates memory.

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

Two per-policy parameters shape this:

``max_steps``
   After this many steps the pipeline returns to the default (first) policy by itself. Use it
   for a motion of a known length.

``interruptible``
   When false, switch requests are rejected while the policy is active. Use it for a motion
   that must not be cut short, such as a jump.

A third one, ``disabled_safety_checks``, skips some safety checks while the policy is active,
for instance a joint velocity bound during a jump; see :ref:`checks-per-policy`.

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

.. _policy-groups:

Policies running together: groups
---------------------------------

Some robots run several policies at the same time on different parts of the body, for
example a locomotion policy on the legs and a manipulation policy on the arms. Give each
policy a ``group``: one policy of every group is active, and all groups step together.

.. code-block:: yaml

   policies: [loco, reach, jump, wave]   # the first policy of each group is its default
   loco:
     group: legs
     # ... stages and model of a lower-body policy, writing the leg joints
   jump:
     group: legs
     max_steps: 147          # back to 'loco', the default of 'legs'
   reach:
     group: arms
     # ... stages and model of an upper-body policy, writing the arm joints
   wave:
     group: arms

.. graphviz::

   digraph groups {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     state [label="State store", shape=cylinder, fillcolor="#fff6d9"];
     subgraph cluster_legs {
       label="Group legs"; style="rounded,dashed"; fontname="Helvetica";
       loco [label="loco (active)", fillcolor="#e8f5e9"];
       jump [label="jump"];
     }
     subgraph cluster_arms {
       label="Group arms"; style="rounded,dashed"; fontname="Helvetica";
       reach [label="reach (active)", fillcolor="#e8f5e9"];
       wave [label="wave"];
     }
     commands [label="Command store\nlegs: leg joints\narms: arm joints", shape=cylinder, fillcolor="#fff6d9"];
     state -> loco;
     state -> reach;
     loco -> commands;
     reach -> commands;
   }

- Each group behaves like the single pipeline described above: switching, ``max_steps``,
  ``interruptible`` and the memory rules all apply within the group. ``switch_policy(wave)``
  changes the arms and leaves the legs walking.
- Groups step one after the other in the order they first appear in ``policies``, on the same
  state and command stores, in the same thread (the inference thread with
  ``async_inference``) and at the same ``policy_rate``.
- Policies of different groups must write different commands. Configuration fails when two of
  them write the same entry, for example the same joint reference, naming both policies.
  Policies of the same group may write the same commands, as before.
- A step succeeds when every group produced commands. A group waiting for data does not stop
  the others from writing theirs, but the controller holds the robot (see :doc:`safety`) until
  every group has produced commands once.
- Without ``group`` every policy belongs to the same unnamed group: the behaviour described in
  the rest of this page.

What groups do not do:

- There is no blending or arbitration between groups: each one owns its joints.
- Each policy observes only what its own preprocessor reads; a lower-body policy trained with
  the arms still in its observation sees the arms moved by the other group, which it may not
  have seen in training.
- The groups share one inference thread and one rate, so the step time is the sum of the
  groups' step times.
