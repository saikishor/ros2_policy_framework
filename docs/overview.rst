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
     rankdir=LR;
     node [shape=box, style="rounded,filled", fillcolor="#eef3fb", fontname="Helvetica", fontsize=11];
     edge [fontname="Helvetica", fontsize=10];

     subgraph cluster_in {
       label="Inputs (shared by all policies)"; style="rounded,dashed"; fontname="Helvetica";
       topics_in [label="Topics\n(InputSource plugins)"];
       ifaces_in [label="State interfaces\n(controller only)"];
     }
     state [label="State store\n(named values)", shape=cylinder, fillcolor="#fff6d9"];

     subgraph cluster_policy {
       label="One policy (there can be several)"; style="rounded,dashed"; fontname="Helvetica";
       pre [label="Preprocessor\nbuilds the observation"];
       inf [label="Inference\nruns the model"];
       post [label="Postprocessor\nturns actions into commands"];
       pre -> inf [label="tensors"];
       inf -> post [label="tensors"];
     }
     commands [label="Command store\n(named values)", shape=cylinder, fillcolor="#fff6d9"];

     subgraph cluster_out {
       label="Outputs (shared by all policies)"; style="rounded,dashed"; fontname="Helvetica";
       topics_out [label="Topics\n(Executor plugins, node)"];
       ifaces_out [label="Command interfaces\n(InterfaceExecutor plugins,\ncontroller)"];
     }

     topics_in -> state;
     ifaces_in -> state;
     state -> pre;
     post -> commands;
     commands -> topics_out;
     commands -> ifaces_out;
   }

Read it left to right: sensors fill the *state store*; the active policy reads it, runs its
three stages and fills the *command store*; executors send the commands to the robot.

The two packages
----------------

``policy_framework``
   The core. It knows nothing about any training framework: the stores, the pipeline that
   runs and switches policies, the two hosts (node and controller), the kinematics model, and
   generic plugins (ONNX inference, topic sources, executors).

``policy_framework_mjlab``
   Everything specific to policies exported by mjlab and pal_mjlab: how their observation is
   laid out, how their actions are scaled, where their gains go, and the configuration and
   launch file for the Kangaroo robot.

To support another training framework you would write a package like the second one, without
changing the first.

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
   * - Step
     - One run of the active policy: preprocessor, inference, postprocessor.
   * - Update
     - One run of the host's loop. In the controller there can be several updates per step.
   * - Plugin
     - A class loaded by name at start-up (pluginlib). Every stage, source and executor is
       one, so new ones can be added from other packages.

Where to go next
----------------

- :doc:`data_flow` follows one number from a sensor to a motor.
- :doc:`hosts` compares the node and the controller, and explains the rates and threads.
- :doc:`policies` explains switching between policies and what each one remembers.
- :doc:`safety` lists what happens when something is missing or goes wrong.
- :doc:`kinematics` explains the robot model available to the policies.
- :doc:`mjlab` explains how mjlab and pal_mjlab policies are run.
- :doc:`extending` shows which plugin to write for what.
