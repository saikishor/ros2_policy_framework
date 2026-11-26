Features
========

What the framework gives you, in short. Each section links to the page that explains it, and
the ``README.md`` at the root of the repository lists every parameter.

One configuration, two hosts
----------------------------

- **Standalone node** (``policy_node``): topics in, topics out. A lifecycle node that
  configures and activates itself, and steps on a timer that follows ``/clock`` with
  ``use_sim_time``.
- **ros2_control controller** (``PolicyController``): state interfaces and/or topics in,
  command interfaces out, inside the control loop.
- The same policy configuration runs in both. Mainly the sources and executors change, plus
  how the gains reach the joints (for example ``gains_as_commands`` for ``PdEffortExecutor``).

See :doc:`hosts`.

Real-time control loop
----------------------

- **Separate rates**: the policy steps at ``policy_rate`` while the executors and the
  kinematics run on every update, for example a 50 Hz policy with a 1 kHz PD law.
- **Inference in its own thread** (``async_inference``): only the policy step runs in a
  ``SCHED_FIFO`` thread, so a slow inference never delays the PD law. The standard
  ros2_control ``is_async`` option is supported too.
- **No allocation in the cycle**: entries are registered while configuring, and the cycle
  only uses indices. All models are loaded at configuration, so switching policies never loads
  a model; the only work on a switch is a log line and the ``~/active_policies`` message.
- **Unreadable state interfaces** (for example locked by an asynchronous hardware component)
  are tolerated for up to ``max_failed_reads`` updates.

See :doc:`hosts`.

Data from anywhere, commands to anywhere
----------------------------------------

- **Named like ros2_control interfaces**: the state store holds named scalars such as
  ``leg_left_1_joint/position`` or ``cmd_vel/linear.x``. Topic sources write the same names as
  the equivalent state interfaces, so moving an input between a topic and a state interface is
  configuration only.
- **Inputs**: joint states, IMU, twist and odometry topic sources. In the controller every
  entry no source provides is claimed as a state interface.
- **Outputs**: ``MultiDOFCommand`` topics from the node, or command interfaces from the
  controller: efforts from a PD law (``PdEffortExecutor``), or references of a chained
  controller such as a pid_controller (``ReferenceInterfaceExecutor``).
- **Backend-independent stages**: the stages exchange named float32 tensors, so only the
  inference plugin knows the backend (``OnnxInference``, ONNX Runtime on the CPU).

See :doc:`data_flow`.

Several policies, switched at runtime
-------------------------------------

- **Load several policies at once** and switch between them with the ``switch_policy``
  service while the robot moves. A switch applies at the start of the next step.
- **See what is loaded and running**: the ``list_policies`` service lists the loaded policies,
  the latched ``active_policies`` topic tells which one runs.
- **Warm memory**: an inactive policy keeps observing every step, so its observation history
  follows the robot and it resumes from the real recent past.
- **Motions that end**: ``max_steps`` returns to the first policy after N steps (for example a
  jump), and ``interruptible: false`` rejects switches while it runs.
- **Gains follow the policy**: each policy's stiffness and damping are written as commands on
  every step, or sent to a pid_controller when it is activated.

See :doc:`policies`.

Safety built in
---------------

- **Hold until ready**: from activation until the policy's first commands, the executors hold
  the robot where it is.
- **Neutral commands**: while ``safety_entry`` is 0, user commands are ignored and the policy
  keeps balancing with neutral commands (for example zero velocity).
- **Command timeouts**: a command that stops arriving falls back to the neutral command after
  its ``timeout``, which catches an operator who stopped sending.
- **Built-in checks**: base tilt, joint position limits from the robot description, and the
  age of the observation (``max_input_age``), plus a ``max_age`` per input source and per
  topic term.
- **Check plugins**: ``StateBoundsCheck`` (bounds with hysteresis and escalation),
  ``JumpCheck`` (sudden changes, such as an encoder glitch), ``EncoderMismatchCheck``
  (absolute and incremental joint positions disagreeing), ``StateValueCheck`` (fault codes,
  such as an actuator mode in error or a false ``is_safe``), and your own ``SafetyCheck``
  plugins. ``MockSafetyCheck`` injects faults from a topic to test how the robot reacts.
- **A reaction per check**: ``neutral_commands``, ``hold``, ``damp``, ``safe`` or ``error``.
  Every check states its own, and a reaction an executor cannot apply is refused while
  configuring. On ``error`` the controller manager activates the ``fallback_controllers``.
- **Reset the safety** with the ``reset_safety`` service: a latched ``hold``, ``damp`` or
  ``safe``, or a persisting ``neutral_commands``, clears once the cause is gone, without
  reactivating the host.
- **Checks per policy**: a policy can disable checks that do not fit it, for example a joint
  velocity bound during a jump.
- **Bad model outputs never reach the robot**: a non-finite output fails the step, which
  applies ``on_policy_failure``.

See :doc:`safety`.

Timing you can see
------------------

- **Time stamps everywhere**: every value carries the time its data was produced, from the
  sensor to the command.
- **Introspection**: the controller publishes the duration of each update and of each of its
  items, of each policy step and of its preprocessor, inference and postprocessor, the age of
  the observation it acted on, and the age of each input of the active policy on its own
  topics, ``/<controller>/introspection_data/{names,values,full}``.

See :doc:`data_flow`.

Whole-body kinematics
---------------------

- **A Pinocchio model of the robot**, built from the robot description and updated from the
  joint state on every update, shared by all policies.
- **Thread-safe reads**: the writer never waits, and with ``async_inference`` the policies get
  a model computed from the same joint state as their step.
- **Sensor helpers**: orientation and angular velocity of any link from an IMU mounted
  elsewhere.

See :doc:`kinematics`.

mjlab policies out of the box
-----------------------------

- **Observation from the ONNX metadata**: ``MjlabPreprocessor`` builds the observation from
  the names, joints and defaults mjlab exports, with the history laid out as in training.
- **pal_mjlab exports** are recognised by their extra metadata (observation joints, per-term
  history lengths, base link).
- **Actions to commands**: ``MjlabPostprocessor`` scales the actions into joint positions and
  hands over the policy's gains.
- **Kangaroo example**: a launch file runs several walking and imitation policies, in
  simulation, as a controller or as a node.

See :doc:`mjlab`.

Easy to extend
--------------

- **Plugins for every stage**: input sources, preprocessors, observation terms, inference,
  postprocessors, executors and safety checks, all loaded with pluginlib.
- **Observation terms**: replace or add a single term of an mjlab observation without writing
  a preprocessor. ``TopicTerm`` owns the subscription, and ``CommandTerm`` handles the gate,
  the timeout and the stamps of a user command so that a term cannot forget them.
- **Validated parameters**: the parameters are declared with generate_parameter_library, and
  an invalid value fails configuration with the parameter name. The observation term
  parameters (``terms.<term>.*``) are declared when the term is configured and checked by its
  code.

See :doc:`extending`.

Current limits
--------------

- Camera images are not supported as model inputs yet.
- The kinematics has no base position or linear velocity in the world (that needs a state
  estimator).
