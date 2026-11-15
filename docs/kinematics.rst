Kinematics
==========

Some observation terms need to know where the links of the robot are: the orientation of the
pelvis when the IMU sits on the torso, or the position of a hand. Instead of every term
loading the robot description and tracking the joints on its own, the framework offers one
shared model.

.. code-block:: yaml

   kinematics:
     enabled: true
     joints: []      # empty: every movable joint

What it is
----------

``Kinematics`` is a `Pinocchio <https://github.com/stack-of-tasks/pinocchio>`_ model of the
whole robot, built from the robot description (URDF). Given the joint positions and
velocities it computes, for every link:

- its pose relative to the root link of the URDF, or relative to any other link;
- its velocity.

The model has a *fixed base*: it knows how the links are placed relative to each other, not
where the robot is in the world. The world is brought in by the IMU: two helpers combine an
IMU measurement with the model to answer *what would this IMU measure if it were mounted on
that other link?*

.. graphviz::

   digraph base_from_imu {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     imu [label="IMU on the torso\norientation, angular velocity"];
     joints [label="Joint positions\nand velocities"];
     model [label="Kinematics\ntorso relative to pelvis", fillcolor="#fff6d9"];
     base [label="Pelvis in the world\norientation, angular velocity", fillcolor="#e8f5e9"];
     joints -> model;
     imu -> base;
     model -> base;
   }

``world_orientation(link, imu_link, imu_orientation)``
   The orientation of a link in the world.

``world_angular_velocity(link, imu_link, gyroscope)``
   The angular velocity of a link in the world, with the motion of the joints between the
   IMU and that link removed.

Who feeds it
------------

When enabled, the joints of the model are registered as state entries
(``<joint>/position`` and ``<joint>/velocity``), so the controller claims them as state
interfaces and the node expects them from an input source. The host updates the model from
those entries: the controller on every update, the node on every tick.

If only some joints have a state, list them in ``joints``; the others are locked at zero.

Who reads it
------------

Policy plugins receive the model in their configuration context (``ctx.kinematics``). They
look up the links they need once, while configuring, and read poses during the cycle.

Other code can read it too, through ``PolicyController::kinematics()``. Reading and updating
are protected by a real-time mutex: the update never waits (if a reader holds the lock, that
one update is skipped and the next one catches up), and a reader only holds the lock for the
time of a copy.

One model or two
----------------

A policy step must read frames that belong to the joint state it is reading. When the step
runs in the control loop that is automatic. With ``async_inference`` the step runs in
another thread on a snapshot of the state, while the control loop moves on. The framework
then keeps two models:

.. graphviz::

   digraph two_models {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     subgraph cluster_rt {
       label="Control loop"; style="rounded,dashed"; fontname="Helvetica";
       live [label="Live state\n(every update)", shape=cylinder, fillcolor="#fff6d9"];
       host_model [label="Host model\nPolicyController::kinematics()"];
       live -> host_model;
     }
     subgraph cluster_thread {
       label="Inference thread"; style="rounded,dashed"; fontname="Helvetica";
       snap [label="State snapshot\n(per policy step)", shape=cylinder, fillcolor="#fff6d9"];
       policy_model [label="Policy model\nctx.kinematics"];
       terms [label="Observation terms", fillcolor="#e8f5e9"];
       snap -> policy_model -> terms;
       snap -> terms;
     }
     live -> snap [label="copied at the\npolicy tick"];
     host_model -> others [label="read by"];
     others [label="Other code", shape=plaintext, fillcolor=none];
   }

A term therefore never has to think about threads: the model it was given always matches the
state it is given.

Limits
------

- No position or linear velocity of the base in the world. That needs a state estimator.
- Joints must be revolute, continuous or prismatic. Lock the others by leaving them out of
  ``joints``.
