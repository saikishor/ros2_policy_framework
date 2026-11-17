Overview
========

The big picture
---------------

A learned policy is a function: it takes an *observation* (numbers describing the robot and
what it is asked to do) and returns an *action* (numbers that become joint commands). Running
it on a robot takes much more than calling that function:

- the observation must be built from sensors exactly as it was built in training;
- the action must be scaled into commands the robot understands;
- all of it must happen on time, every cycle, next to a fast joint control loop;
- and it must stay safe when something is missing, late or wrong.

The framework splits this work into small pieces with one job each. The policy-specific
pieces are in the middle and never see where the data comes from or goes to. The transport
pieces are at both ends and never see the model.

.. graphviz::

   digraph architecture {
     rankdir=TB;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];

     subgraph cluster_in {
       label="Inputs (shared by all policies)"; style="rounded,dashed"; fontname="Helvetica";
       topics_in [label="Topics\n(InputSource plugins)"];
       ifaces_in [label="State interfaces\n(controller only)"];
     }
     state [label="State store\n(named values)", shape=cylinder, fillcolor="#fff6d9"];
     kin [label="Kinematics\n(optional robot model)"];

     subgraph cluster_policy {
       label="One policy (there can be several)"; labelloc=b; style="rounded,dashed"; fontname="Helvetica";
       pre [label="Preprocessor\nbuilds the observation"];
       inf [label="Inference\nruns the model"];
       post [label="Postprocessor\nturns actions into commands"];
       pre -> inf [label="tensors"];
       inf -> post [label="tensors"];
     }
     commands [label="Command store\n(named values)", shape=cylinder, fillcolor="#fff6d9"];
     safety [label="Safety checks\n(every cycle)", fillcolor="#fde8e8"];

     subgraph cluster_out {
       label="Outputs (shared by all policies)"; labelloc=b; style="rounded,dashed"; fontname="Helvetica";
       topics_out [label="Topics\n(Executor plugins, node)"];
       ifaces_out [label="Command interfaces\n(InterfaceExecutor plugins,\ncontroller)"];
     }

     topics_in -> state;
     ifaces_in -> state;
     state -> pre;
     state -> kin;
     kin -> pre [label="link poses", style=dashed];
     post -> commands;
     commands -> topics_out;
     commands -> ifaces_out;
     state -> safety;
     safety -> pre [label="neutral user commands,\nor not stepped", style=dashed];
     safety -> topics_out [label="node: stop sending", style=dashed];
     safety -> ifaces_out [label="controller: hold, damp,\nsafe or error", style=dashed];
   }

Read it top to bottom: sensors fill the *state store*; the active policy reads it, runs its
three stages and fills the *command store*; executors send the commands to the robot. The
optional kinematics model gives the policies the poses of the robot's links
(:doc:`kinematics`). The safety checks watch the state store on every cycle. On a fault,
either the policy keeps running with neutral user commands, or it is no longer stepped: the
node stops sending, and the controller's executors hold, damp or zero the joints, or hand
over to the fallback controllers (:doc:`safety`).

The two packages
----------------

``policy_framework``
   The core. It knows nothing about any training framework: the stores, the pipeline that
   runs and switches policies, the two hosts (node and controller), the kinematics model, and
   generic plugins (ONNX inference, topic sources, executors), and the safety layer: the
   built-in monitor checks, the ``SafetyCheck`` plugins and the fault reactions.

``policy_framework_mjlab``
   Everything specific to policies exported by mjlab and pal_mjlab: how their observation is
   laid out, how their actions are scaled, where their gains go, and the configuration and
   launch file for the Kangaroo robot.

To support another training framework you would write a package like the second one, without
changing the first.

Where the files are
-------------------

``policy_framework`` groups its files by the role they play. The same folders appear under
``include/policy_framework/`` (headers), ``src/`` (sources), ``test/`` (tests) and
``params/`` (parameter definitions), so a class, its implementation, its test and its
parameters are always found under the same path.

.. code-block:: text

   policy_framework/
     include/policy_framework/
       interfaces/      what plugins derive from
       core/            what the hosts are built on
       hosts/           the programs that run the cycle
       plugins/
         sources/       where the data comes from
         inference/     what runs the model
         stages/        generic preprocessors and postprocessors
         executors/     where the commands go
         safety/        SafetyCheck plugins
       plugin_interfaces.hpp
     src/  test/  params/   same folders as above
     test/fixtures/         test models and URDF
     msg/  srv/             MockFault, SwitchPolicy

.. list-table::
   :header-rows: 1
   :widths: 18 52 30

   * - Folder
     - Content
     - Example include
   * - ``interfaces/``
     - One header per plugin base class (``Inference``, ``Preprocessor``, ``Postprocessor``,
       ``InputSource``, ``Executor``, ``InterfaceExecutor``, ``SafetyCheck``, the observation
       terms) and the types they share (``types.hpp``, ``plugin_context.hpp``). This is the
       API a plugin package depends on.
     - ``policy_framework/interfaces/preprocessor.hpp``
   * - ``core/``
     - The pieces the hosts are built on: the stores, the pipeline, the kinematics model and
       the safety monitor.
     - ``policy_framework/core/data_store.hpp``
   * - ``hosts/``
     - The runner and the two hosts: standalone node and ros2_control controller.
     - ``policy_framework/hosts/policy_controller.hpp``
   * - ``plugins/<kind>/``
     - The generic plugins shipped with the framework, one folder per kind of plugin.
     - ``policy_framework/plugins/sources/topic_sources.hpp``

``policy_framework/plugin_interfaces.hpp`` includes every interface except the observation
terms, for a plugin that needs several of them.

``policy_framework_mjlab`` is small and stays flat (``include/``, ``src/``, ``test/``,
``params/``). Its test-only plugins are in ``test/plugins/``, and the deployment
configuration has one folder per robot (``config/kangaroo/``) next to the ``launch/`` files.

A new file goes in the folder of its role. For example, a new input source plugin goes in
``plugins/sources/`` in all four trees: header, source, test and parameters.

Vocabulary
----------

These words are used with one meaning throughout the code and the documentation.

.. list-table::
   :header-rows: 1
   :widths: 22 78

   * - Term
     - Meaning
   * - Policy
     - A trained model together with the three stages that run it: a preprocessor, an
       inference and a postprocessor. It has a name, for example ``loco``.
   * - Observation
     - The input of the model: one flat vector of numbers, built every step.
   * - Observation term
     - One named block of the observation, for example ``joint_pos`` or ``base_ang_vel``.
       The observation is its terms one after another.
   * - History
     - The past values of a term that the model also receives. A term with a history of 3
       appears as its three most recent values.
   * - Action
     - The raw output of the model. It is not a joint command yet.
   * - State store
     - A table of named numbers holding the latest inputs, for example
       ``leg_left_1_joint/position``.
   * - Command store
     - The same kind of table holding the latest outputs, for example
       ``leg_left_1_joint/position`` (a position reference) or ``leg_left_1_joint/stiffness``.
   * - Entry
     - One named number in a store, with the time it was produced.
   * - Input source
     - A plugin that copies a topic into the state store.
   * - Executor
     - A plugin that sends the command store to the robot.
   * - Host
     - The program that runs the cycle: the standalone node or the ros2_control controller.
   * - Runner
     - The part shared by both hosts. It loads the plugins from the parameters and owns the
       stores and the pipeline.
   * - Pipeline
     - Holds all the policies, runs the active one and switches between them.
   * - Group
     - A set of policies with one active policy, running together with the other groups
       (e.g. legs and arms); each group is a pipeline. By default there is one group.
   * - Step
     - One run of the active policy of every group: preprocessor, inference, postprocessor.
   * - Update
     - One run of the host's loop. In the controller there can be several updates per step.
   * - Plugin
     - A class loaded by name at start-up (pluginlib). Every stage, source, executor and
       safety check is one, so new ones can be added from other packages.
   * - User command
     - An input a person or a planner sends, such as a velocity command or a motion
       reference. It is replaced by a *neutral command* (zero velocity, a still stand) when
       it times out or must be ignored.
   * - Safety check
     - A test of the state on every cycle: built into the ``SafetyMonitor`` (tilt, joint
       limits, input age) or a ``SafetyCheck`` plugin. It returns a ``Verdict``: safe or
       not, and why.
   * - Fault reaction
     - What a failing check makes the host do: ``neutral_commands``, ``hold``, ``damp``,
       ``safe`` or ``error``. Every reaction but ``neutral_commands`` is *latched*: it stays
       until the host is reactivated.
   * - Safety entry
     - A state entry that is 0 while user commands must be ignored, for example a dead-man
       switch. ``CommandGate`` applies it, and the ``neutral_commands`` reaction.

Where to go next
----------------

- :doc:`data_flow` follows one number from a sensor to a motor.
- :doc:`hosts` compares the node and the controller, and explains the rates and threads.
- :doc:`policies` explains switching between policies and what each one remembers.
- :doc:`safety` lists what happens when something is missing or goes wrong.
- :doc:`kinematics` explains the robot model available to the policies.
- :doc:`mjlab` explains how mjlab and pal_mjlab policies are run.
- :doc:`extending` shows which plugin to write for what.
