mjlab policies
==============

``policy_framework_mjlab`` runs policies trained with mjlab and exported to ONNX. The export
stores, next to the network, *metadata* describing how it must be used: the joints, the
observation terms, the action scale, the gains. The two stages of this package read that
metadata, so a policy needs almost no configuration: point ``model_path`` at the file, or at
a directory holding it (its first ``.onnx`` file is used).

.. graphviz::

   digraph mjlab {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     onnx [label="ONNX file\nnetwork + metadata", shape=note, fillcolor="#fff6d9"];
     pre [label="MjlabPreprocessor\nobservation_names, command_names,\njoint_names, defaults\n(+ pal_mjlab keys)"];
     inf [label="OnnxInference"];
     post [label="MjlabPostprocessor\naction_scale, defaults,\nstiffness, damping\n(+ action_joint_names, action_offset)"];
     onnx -> pre [style=dashed, label="metadata"];
     onnx -> inf [style=dashed, label="network"];
     onnx -> post [style=dashed, label="metadata"];
     pre -> inf [label="obs"];
     inf -> post [label="actions"];
   }

Building the observation
------------------------

The metadata lists the observation terms in order (``observation_names``). The preprocessor
creates one *term* object per name. Each step, every term writes its current value; the
preprocessor keeps the history and lays everything out in the input tensor.

.. graphviz::

   digraph observation {
     rankdir=TB;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     state [label="State store", shape=cylinder, fillcolor="#fff6d9"];
     prev [label="Previous outputs", shape=cylinder, fillcolor="#fff6d9"];
     topic [label="Velocity policy: state store\n(TwistSource) or a topic\n(TwistCommandTerm)", shape=plaintext, fillcolor=none];
     motion [label="Motion policy: previous\njoint_pos + joint_vel outputs", shape=plaintext, fillcolor=none];
     t1 [label="base_ang_vel\ncurrent value (3)"];
     t2 [label="joint_pos\ncurrent value (n)"];
     t3 [label="actions\ncurrent value (m)"];
     t4 [label="command\ncurrent value\n(3, or 2n for a motion)"];
     h [label="History of each term\noldest ... newest", fillcolor="#f3e9fb"];
     obs [label="obs tensor\n[ base_ang_vel history | joint_pos history | actions history | command history ]",
          fillcolor="#e8f5e9"];
     state -> t1;
     state -> t2;
     prev -> t3;
     topic -> t4;
     motion -> t4;
     t1 -> h; t2 -> h; t3 -> h; t4 -> h;
     h -> obs;
   }

Three things are worth knowing about this layout:

- **Term-major.** The tensor holds all the frames of the first term, then all the frames of
  the second term, and so on. Within a term the oldest frame comes first.
- **Back-fill.** On the first step after a clear, the whole history of each term is filled
  with its current value. mjlab does the same after an environment reset.
- **History length.** For mjlab exports it is the same for every term, deduced from the size
  of the input. pal_mjlab exports give one length per term.

The terms
---------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Term
     - Value
   * - ``base_ang_vel``, ``imu_ang_vel``
     - Angular velocity measured by the IMU
   * - ``base_lin_acc``
     - Linear acceleration measured by the IMU
   * - ``projected_gravity``, ``imu_projected_gravity``
     - Direction of gravity in the IMU frame, from the IMU orientation
   * - ``base_projected_gravity``
     - Direction of gravity in the base frame (pal_mjlab, see below)
   * - ``base_lin_vel``
     - Linear velocity from an odometry source
   * - ``joint_pos``
     - Joint positions minus their default positions
   * - ``joint_vel``
     - Joint velocities
   * - ``actions``
     - The raw action of the previous step
   * - ``command``, ``velocity_command``
     - The velocity the user asks for: forward, sideways, turning
   * - ``command`` of a motion policy (``command_names`` is ``motion``)
     - The joint positions and velocities the policy itself predicted at the previous step;
       the measured joint state on the first step after it becomes active

Any term can be replaced, and new ones added, without touching the preprocessor: set
``<policy>.terms.<name>.type`` to an ``ObservationTerm`` plugin, e.g. to read the velocity
command straight from its topic:

.. code-block:: yaml

   loco:
     terms:
       command:
         type: policy_framework_mjlab/TwistCommandTerm
         topic: /cmd_vel
         timeout: 1.0

A topic term subscribes once per policy. When many policies take the same command, use one
``TwistSource`` on the host instead (``sources: [cmd_vel]``) and leave the built-in ``command``
term: every policy reads the shared ``cmd_vel/*`` entries from a single subscription. The
source name must match the preprocessor's ``velocity_command_name`` (default ``cmd_vel``).
The Kangaroo configuration does this.

Extra inputs
------------

Every model input other than the observation (``observation_input``) is filled by an
``ExtraInput`` plugin: ``<policy>.extra_inputs.<input>.type`` when set, else the built-in one of
that name. An input with neither fails the configuration, unless
``<policy>.unknown_extra_inputs`` is ``ignore`` (it then stays at zero).

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Input
     - Built-in
     - Value
   * - ``time_step``
     - ``policy_framework_mjlab/TimeStepInput``
     - The steps since the policy became active
   * - ``obstacle_height``
     - ``policy_framework_mjlab/ParameterInput``
     - ``<policy>.extra_inputs.obstacle_height.value``; unset: the ``reference_obstacle_height``
       metadata

``ParameterInput`` fits any scalar input (parameters ``value`` and ``metadata_default``, default
``reference_<input>``). Its value is taken when the policy becomes active, so a running motion
keeps it and a change at runtime applies to the next one:

.. code-block:: bash

   ros2 param set /policy_node imitation_v30.extra_inputs.obstacle_height.value 0.45

pal_mjlab exports
-----------------

pal_mjlab adds metadata that the stages recognise:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Metadata
     - Meaning
   * - ``observation_joint_names``
     - The joints the policy observes, when it does not observe all of them
   * - ``action_joint_names``, ``action_offset``
     - The joints the policy drives, and the offset of each action
   * - ``observation_terms_dim``
     - The size of each term, checked against what the terms produce
   * - ``observation_terms_history_length``
     - The history length of each term
   * - ``goal_root_body``
     - The link the policy calls its *base*

The last one matters because the IMU is not always mounted on the base. With
``goal_root_body`` set, the terms ``base_ang_vel`` and ``base_projected_gravity`` describe
that link: the IMU measurement is moved to it with the :doc:`kinematics`. This needs
``kinematics.enabled`` and the parameter ``imu_frame`` (the URDF link of the IMU).
``base_projected_gravity`` cannot be used without ``goal_root_body``.

From actions to commands
------------------------

The postprocessor writes, for each joint the policy drives (``action_joint_names``, or the
metadata joints minus ``excluded_joints``)::

   <joint>/<position_interface> = action_scale * action + offset
   <joint>/<velocity_interface> = 0      (not written when velocity_interface is empty)

``position_interface`` and ``velocity_interface`` default to ``position`` and ``velocity``.

These are *references* for a joint PD loop. The gains of that loop are part of the policy
(``joint_stiffness``, ``joint_damping`` in the metadata) and reach it in one of two ways:

.. graphviz::

   digraph gains {
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];
     post [label="MjlabPostprocessor\ngains of the policy"];
     subgraph cluster_a {
       label="gains_as_commands (e.g. Kangaroo controller modes)"; style="rounded,dashed"; fontname="Helvetica";
       cmd [label="Command entries\n<joint>/stiffness\n<joint>/damping", shape=cylinder, fillcolor="#fff6d9"];
       pd [label="PdEffortExecutor\nPD law, every update", fillcolor="#e8f5e9"];
       cmd -> pd;
     }
     subgraph cluster_b {
       label="gains_controller (e.g. Kangaroo node mode)"; style="rounded,dashed"; fontname="Helvetica";
       params [label="Parameters\ngains.<joint>.p / .d"];
       pid [label="pid_controller\nin ros2_control", fillcolor="#e8f5e9"];
       params -> pid;
     }
     post -> cmd [label="every step, and before the\nfirst one (initial_commands,\ndefault policy)"];
     post -> params [label="when the policy\nbecomes active"];
   }

``gains_as_commands``
   The gains travel with the references, as command entries. A switch of policy changes the
   gains in the same step as the references. They are also known before the first step, which
   lets the executor hold the robot (see :doc:`safety`).

``gains_controller``
   The gains are set as parameters of a separate ``pid_controller`` each time the policy
   becomes active. They are sent from a 0.5 s timer, which waits for that controller's
   parameter services; only the latest request is sent, and a rejected parameter is logged,
   not retried. Until they arrive, the new references run with the previous gains.

The Kangaroo example
--------------------

``kangaroo_mjlab.launch.py`` runs walking policies (``loco_<version>``) and motion policies
(``imitation_<version>``), one per version given in ``loco_models`` and ``imitation_models``.
The first loco version is the default policy: every imitation returns to it after its
``imitation_frames``, even when another loco version was active before the motion.
The policies can be listed in a YAML file passed as ``config`` instead, each naming its
``model`` as ``<task>/<version>``, with any policy name and per-policy settings on top of
``config/kangaroo/policies.yaml``; see ``config/kangaroo/examples``. The models are looked up
in ``models_dir``, by default ``models/simple`` of ``pal_rl_policies``.

It runs in three modes:

.. list-table::
   :header-rows: 1
   :widths: 24 76

   * - ``mode``
     - What runs
   * - ``controller``
     - ``PolicyController``: policy at 50 Hz, ``PdEffortExecutor`` at 1 kHz writing joint
       efforts, gains as commands
   * - ``controller_async``
     - The same with ``async_inference``
   * - ``node``
     - ``policy_node`` publishing references to a ``pid_controller`` that receives the gains
       as parameters

The settings of each kind of policy are written once, in ``config/kangaroo/policies.yaml``, and
shared by the three modes; the launch copies them into one policy per version. Apart from
the host-specific file, only the gains differ: in the controller modes the launch sets
``gains_controller`` to empty and ``gains_as_commands`` to true on every policy. The README
has the commands.
