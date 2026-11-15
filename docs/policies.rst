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

Two per-policy parameters shape this:

``max_steps``
   After this many steps the pipeline returns to the default (first) policy by itself. Use it
   for a motion of a known length.

``interruptible``
   When false, switch requests are rejected while the policy is active. Use it for a motion
   that must not be cut short, such as a jump.

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
