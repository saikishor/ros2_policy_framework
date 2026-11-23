# ros2_policy_framework

Run learned policies (e.g. mjlab-trained ONNX policies) on ROS 2 robots, with a clear split
between **where data comes from**, **what the policy does with it** and **where commands go**.
The same policy configuration runs

- as a **standalone node**: topics in, topics out; or
- as a **ros2_control controller**: state interfaces and/or topics in, command interfaces out,
  in the control loop, with inference in its own thread (`async_inference`) or fully
  asynchronous (`is_async`),

and several policies can be loaded at once and **switched at runtime**.

This README is the reference (parameters, plugins, behaviour). The concepts, with diagrams of
how the data flows and how the pieces fit, are explained in [`docs/`](docs/index.rst); the
features are summarised in [`docs/features.rst`](docs/features.rst).

## Packages

| Package | Content |
|---------|---------|
| `policy_framework` | Core: plugin base classes (incl. `ObservationTerm`, `TopicTerm`, `CommandTerm`), `DataStore`, `PolicyPipeline`, `PolicyRunner`, `Kinematics`, `PolicyNode` (standalone), `PolicyController` (ros2_control), generic plugins (ONNX inference, topic sources, MultiDOFCommand executor, PD effort and reference interface executors, concat/split stages, `StateBoundsCheck`, `JumpCheck` and `MockSafetyCheck` safety checks), `SafetyCheck` plugin interface, `SafetyMonitor` (built-in state checks), `SwitchPolicy` service, `MockFault` message |
| `policy_framework_mjlab` | Pre/postprocessor plugins for mjlab and pal_mjlab policies (observation terms with history, action scaling, per-policy gains), the `TwistCommandTerm` observation term and the Kangaroo deployment config/launch |

### Repository layout

`policy_framework` groups its files by role, with the same folders under `include/policy_framework/`,
`src/`, `test/` and `params/` (parameter definitions):

| Folder | Content | Example include |
|--------|---------|-----------------|
| `interfaces/` | Plugin base classes, one header per kind (`Inference`, `Preprocessor`, `Postprocessor`, `InputSource`, `Executor`, `InterfaceExecutor`, `SafetyCheck`, observation terms) and their shared types (`types.hpp`, `plugin_context.hpp`) | `policy_framework/interfaces/preprocessor.hpp` |
| `core/` | What the hosts are built on: `DataStore`, `PolicyPipeline`, `Kinematics`, `SafetyMonitor`, `LatestMessage` | `policy_framework/core/data_store.hpp` |
| `hosts/` | `PolicyRunner`, `PolicyNode` (standalone), `PolicyController` (ros2_control) | `policy_framework/hosts/policy_controller.hpp` |
| `plugins/<kind>/` | The generic plugins: `sources/`, `inference/`, `stages/`, `executors/`, `safety/` | `policy_framework/plugins/sources/topic_sources.hpp` |

`policy_framework/plugin_interfaces.hpp` includes every interface except the observation terms.
Test models and URDF are in `policy_framework/test/fixtures/`.

`policy_framework_mjlab` is small and stays flat. It has its test-only term plugins in
`test/plugins/` and one configuration folder per robot (`config/kangaroo/`).

## Architecture

```
                 shared by all policies                 one per policy                 shared
 ┌──────────────────────────────────┐   ┌────────────────────────────────────────┐   ┌─────────────────────────┐
 │ InputSource plugins (topics)  ─┐ │   │ Preprocessor → Inference → Postprocessor│   │ node: Executor plugins  │
 │ state interfaces (controller) ─┴─┼──▶│   state ──▶ TensorMap ──▶ TensorMap ──▶  ├──▶│  (topics)               │
 │            state DataStore       │   │                             commands   │   │ controller: Interface-  │
 │                │                 │   │                                        │   │  Executor plugins       │
 └────────────────┼─────────────────┘   └────────────────────────────────────────┘   └─────────────────────────┘
                  ▼
   safety: SafetyMonitor + SafetyCheck plugins ──▶ fault reaction (neutral commands, hold, damp, safe, error)
```

- **DataStore**: flat table of named scalars, named like ros2_control interfaces
  (`leg_left_1_joint/position`, `torso_imu_sensor/orientation.x`, `cmd_vel/linear.x`). Topic
  sources write the names the equivalent state interfaces have, so moving an input between a
  topic and a state interface is configuration only. Entries are registered while configuring;
  the control cycle only uses indices.
- **Preprocessor**: state store → model inputs. It also gets the previous step's outputs
  (last action, autoregressive motion reference) and owns the observation history.
  `MjlabPreprocessor` takes **`ObservationTerm` plugins**, fed by the state store or directly
  by a topic.
- **Inference**: runs the model on named float32 tensors (`TensorMap`), so the other stages do
  not depend on the backend (`OnnxInference`: ONNX Runtime, CPU).
- **Postprocessor**: model outputs → named commands (`<joint>/position`, ...).
- **Executors**: send the commands. Node: `Executor` plugins publishing topics. Controller:
  `InterfaceExecutor` plugins computing command interface values, e.g. a PD law into efforts
  (`PdEffortExecutor`) or references of a chained controller (`ReferenceInterfaceExecutor`).

One cycle: [controller: state interfaces] → sources → kinematics → safety checks →
preprocessor → inference → postprocessor → term input ages (`max_age`) → executors. While an
input was never received the preprocessor skips the step: the node sends nothing, the
controller keeps the last commands (see Safety). A failing safety check applies its reaction
(see `docs/safety.rst`); while one other than `neutral_commands` is latched the policy is not
stepped. A stale term input latches its reaction too, and that step's commands are not
applied.

## Running

### Standalone node

```bash
ros2 run policy_framework policy_node --ros-args --params-file my_policies.yaml
```

A lifecycle node that configures and activates itself (`autostart`). Every policy input must
come from an input source (configuration fails listing the missing ones); commands leave
through topic executors. The policy steps on a timer at `update_rate`, which follows `/clock`
with `use_sim_time`. Crashes print a stack trace (`backward_ros`). The safety checks run on
every tick: a fault with a latched reaction, or a failed policy step, stops the commands until
the node is reactivated (the node does not own the hardware); neutral commands keep the policy
running.

### ros2_control controller

```yaml
controller_manager:
  ros__parameters:
    policy_controller:
      type: policy_framework/PolicyController

policy_controller:
  ros__parameters:
    update_rate: 1000        # executors (PD law) run on every update
    policy_rate: 50.0        # see Rates
    async_inference: true    # see Async inference
    sources: []              # anything not provided by a source is claimed as state interface
    executors: [pd]
    pd:
      type: policy_framework/PdEffortExecutor   # effort = kp (q_ref - q) + kd (dq_ref - dq)
    # ... policies as below
```

or, chaining into a pid_controller that closes the loop:

```yaml
    update_rate: 50
    executors: [reference]
    reference:
      type: policy_framework/ReferenceInterfaceExecutor
      prefix: rl_impedance_controller   # writes rl_impedance_controller/<joint>/position|velocity
```

- **Inputs**: every state entry no input source provides is claimed as the state interface of
  the same name, so topics, state interfaces or a mix all work. Feedback an executor needs on
  every update (e.g. PD joint positions/velocities) must come from state interfaces.
- **Outputs**: each executor claims its command interfaces (`PdEffortExecutor`:
  `<joint>/effort`; `ReferenceInterfaceExecutor`: `<prefix>/<entry>`); two executors cannot
  claim the same one. Topic executors are rejected (node only).
- **Rates**: the policy steps every `update_rate / policy_rate` updates (an integer; 0: every
  update), and topic sources are read on those steps. Executors and the kinematics run on every
  update, e.g. a 1 kHz PD law with a 50 Hz policy.
- **Async inference** (`async_inference: true`, recommended with a PD law): only the policy
  step runs in its own real-time thread (`SCHED_FIFO` `inference_thread_priority`, below the
  controller manager's 50). Each policy tick hands over a copy of the state and triggers the
  step without waiting; its commands apply from the first update after it finished, and a tick
  that finds the step still running, or the thread busy for an instant, is skipped with a
  warning. Without real-time permissions
  the thread uses the default scheduler.
- **is_async**: the standard ros2_control option moves the whole `update()` to its own thread.
  Suits a reference executor, not a PD law. Exclusive with `async_inference`.
- **Safety**:
  - *Hold*: from activation until the active policy produced its first commands (it may wait
    for an input, or for its first inference) the executors hold the robot where it is:
    `PdEffortExecutor` runs its PD law around the joint positions at activation, with the gains
    the policy provides before its first step (`gains_as_commands`) or its `stiffness`/`damping`
    parameters; `ReferenceInterfaceExecutor` writes nothing (the chained controller holds).
    After that, a step waiting for data keeps the last commands.
  - *Safe values*: on deactivation each executor writes its safe values: zero effort for
    `PdEffortExecutor`, nothing for `ReferenceInterfaceExecutor`.
  - *Fault reactions*: the safety checks run on every update, before the policy. A failing
    check applies its reaction; a failed policy step (including a non-finite model output,
    which never reaches the commands) applies `on_policy_failure`. While a reaction other than
    `neutral_commands` is latched the policy is neither stepped nor followed: the executors
    write `hold`, `damp` or `safe` on every update until the controller is reactivated, or for
    `error` they write their safe values and `update()` returns `ERROR` (the controller manager
    deactivates the controller and activates its `fallback_controllers`). A reaction an
    executor cannot apply is refused while configuring. See `docs/safety.rst`.
  - *Unreadable state interfaces* (locked by their writer, e.g. an asynchronous hardware
    component): the previous value is used for up to `max_failed_reads` consecutive updates;
    one more writes the safe values and returns `ERROR`.
  - *Safety entry* (`safety_entry`, e.g. `rl_safety_controller/safe`): a state entry, claimed
    as a state interface, that is 0 while user commands must be ignored. The policy keeps
    running (the robot keeps balancing) with neutral commands: zero velocity (`TwistSource`,
    `TwistCommandTerm`). Transitions are logged. The same
    neutral commands are used while the runner's `set_neutral_commands()` is on (the
    `NEUTRAL_COMMANDS` fault reaction; see `docs/safety.rst`).
- **Introspection**: the controller publishes its own pal_statistics topic,
  `/<controller>/introspection_data/{names,values,full}` (not the controller manager's
  shared `introspection_data`), on every update:
  - `step_time_us`: total duration of the last policy step;
  - `preprocessor_time_us`, `inference_time_us`, `postprocessor_time_us`: its stages
    (0 for a stage that did not run, e.g. while waiting for data). Their sum is at most
    `step_time_us`, which also covers switching and the inactive policies' `observe()`;
  - `observation_age_ms`: age of the oldest input of that step when its commands were
    applied;
  - `input_age_ms.<policy>.<input>`: age of each input (a term, or an entry for
    `ConcatPreprocessor`; 0 for the inputs of an inactive policy).

  Also available as `PolicyController::timing()` and `PolicyController::input_age_ms()`.
  For example: `ros2 topic echo /policy_controller/introspection_data/full`.

  > Changed: these values used to be on ros2_control's introspection as
  > `<controller>.<name>`, with `policy_step_time_us` now named `step_time_us`.

## Configuration

### Parameters

All parameters are declared and validated with
[generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library)
(definitions in the `params/` YAML files of each package), read at configure time and read-only, except
the on-demand `<policy>.terms.<term>.type`. Invalid values fail configuration with the parameter
name. Write floating point values with a decimal point (`50.0`).

| Parameter | Description |
|-----------|-------------|
| `sources` | Input source names; each needs `<source>.type`, may have `<source>.max_age` (s, default 0: off; a fault when its latest message arrived longer ago, on the receive time) with the reaction `<source>.max_age_on_fault` (default `error`); not for a command source, whose timeout handles a stopped operator. Covers input sources only: state interfaces are fresh by construction, and an observation term with its own subscription has its own `max_age` (see [Writing plugins](#writing-plugins)) |
| `policies` | Policy names (required); the first one is the default |
| `<policy>.preprocessor` / `.postprocessor` | Plugin classes (required) |
| `<policy>.inference` | Plugin class, default `policy_framework/OnnxInference` |
| `<policy>.max_steps`, `.interruptible` | Switching, see below |
| `<policy>.disabled_safety_checks` | Safety checks skipped while this policy is active (default none), e.g. a joint velocity bound for a jump: names from `safety_checks`, or the monitor checks `tilt`, `joint_position_limits`, `max_input_age`; an unknown name is refused. A disabled check is reset when it applies again; a latched fault stays latched across switches; a source's or term's `max_age` always applies. See [Checks per policy](docs/safety.rst) |
| `<policy>.<plugin param>` | Parameters of the policy's plugins |
| `<policy>.terms.<term>.max_age`, `.max_age_on_fault` | A `TopicTerm`: fault when its latest message arrived longer ago than this, in s, on the receive time (default 0: off), checked after each step of its policy; reaction `hold`, `damp`, `safe` or `error` (default `error`). Refused by a `CommandTerm` (it has its `timeout`). See [Writing plugins](#writing-plugins) |
| `executors` | Executor names; each needs `<executor>.type` (`Executor` in the node, `InterfaceExecutor` in the controller) |
| `kinematics.enabled`, `kinematics.joints` | Whole-body kinematics (default false), see below |
| `safety_entry` | State entry that is 0 while user commands must be ignored (default empty: always accepted), see Safety |
| `safety.max_tilt`, `safety.imu` | Fault when the IMU z axis tilts more than this from the vertical, in rad (default 0: off) |
| `safety.joint_position_limits`, `safety.position_margin` | Fault when a commanded joint leaves its robot description limits narrowed by the margin (default false, 0) |
| `safety.max_input_age` | Fault when the observation of the last step is older than this, in s (default 0: off) |
| `safety.tilt_on_fault`, `.joint_position_on_fault`, `.input_age_on_fault` | Reaction to each of these checks (default `error`): `neutral_commands`, `hold`, `damp`, `safe`, `error`, see `docs/safety.rst` |
| `safety_checks` | `SafetyCheck` plugin names; each needs `<check>.type` and has a reaction `<check>.on_fault` (default `error`). There is no global default reaction: every check states its own |
| `on_policy_failure` | Reaction to a failed policy step, including a non-finite model output (default `error`): `hold`, `damp`, `safe` or `error` (not `neutral_commands`: the policy itself failed) |
| `policy_rate` | Controller: policy rate in Hz (default 0 = every update) |
| `max_failed_reads` | Controller: consecutive updates with an unreadable state interface that are tolerated (default 5) |
| `async_inference`, `inference_thread_priority` | Controller: policy step in its own thread (default false), its priority (default 40) |
| `update_rate` | Node: policy rate in Hz (default 50) |
| `autostart` | Node: configure and activate on startup (default true) |
| `robot_description` | Node: URDF for the kinematics (the controller uses the controller manager's) |

### Policy switching

```bash
ros2 service call /<node or controller>/switch_policy policy_framework/srv/SwitchPolicy "{policy_name: jump}"
```

A switch applies at the start of the next step and calls `reset()` on the activated policy's
stages; requesting the active policy restarts it. A policy keeps its memory (history, previous
outputs) when switched away and back, as `pal_policy_deployer` does, and while it is not active
its preprocessor still `observe()`s every step, so its observation history follows the robot
and it resumes from the real recent past. The memory is cleared only before its first
activation and on every (re)activation of the node/controller, which also restarts the first
policy. Per policy, `max_steps` (default 0 = unlimited) returns to the first policy
after N steps (e.g. a jump motion), and `interruptible: false` rejects switches while it runs.
All models are loaded at configuration: switching never loads or allocates.

### Kinematics

```yaml
kinematics:
  enabled: true
  joints: []   # joints fed from the state; empty: every movable joint, others locked at zero
```

A [Pinocchio](https://github.com/stack-of-tasks/pinocchio) model of the whole robot (fixed base
at the URDF root link), shared by all policies through `PluginContext::kinematics` and updated
from `<joint>/position` and `<joint>/velocity` (registered as state entries, so the controller
claims them) on every controller update or node tick, before the policy step, once all were
received. Its readers are thread-safe; see [Using the kinematics](#using-the-kinematics).
With `async_inference` the policies get a second model, computed in the inference thread from
the state snapshot of their step, so the frames a term reads always belong to the joint state
it reads; `PolicyController::kinematics()` keeps following the state interfaces on every update.
Limits: no base position or linear velocity in the world (needs a state estimator); joints
must be revolute, continuous or prismatic (lock the others).

### Plugins

| Plugin | Type | Parameters (under `<name>.`) |
|--------|------|------------------------------|
| `policy_framework/OnnxInference` | Inference | `model_path` (file, or directory: its first `.onnx` by name), `num_threads` (1); float32 tensors, dynamic dimensions fixed to 1 |
| `policy_framework/JointStateSource` | InputSource | `topic`, `joints` (empty = any) → `<joint>/position\|velocity\|effort` |
| `policy_framework/ImuSource` | InputSource | `topic`, `sensor_name` (source name) → `<sensor>/orientation.*`, `angular_velocity.*`, `linear_acceleration.*` |
| `policy_framework/TwistSource` | InputSource | `topic`, `timeout` (1.0 s, above zero; on the receive time) → `<name>/linear.*`, `angular.*`, stamped now; zero when absent or stale |
| `policy_framework/OdometrySource` | InputSource | `topic` → `<name>/position.*`, `orientation.*`, `linear.*`, `angular.*` |
| `policy_framework/MultiDofCommandExecutor` | Executor (node) | `topic`, `values_interface` (`position`), `values_dot_interface` (`velocity`, empty: 0); drops a message while the previous one is being published |
| `policy_framework/PdEffortExecutor` | InterfaceExecutor | `joints` (default: every `<joint>/position` command), `effort_interface` (`effort`), `stiffness`/`damping` (per joint, used when the policy writes no `<joint>/stiffness`/`damping` commands), `effort_limits` (per joint, 0 or empty: none), `fault_damping` (per joint, default empty; the `DAMP` fault reaction writes `-fault_damping · velocity` within the effort limits, supported only when every value is above zero). Fault reactions: `HOLD` PD around the position at the fault, `DAMP`, `SAFE` zero effort; a joint with a missing or non-finite value is left unchanged |
| `policy_framework/StateBoundsCheck` | SafetyCheck | `prefixes`, `interfaces` → `<prefix>/<interface>`, `<interface>.min`/`.max` (at least one), `<interface>.hysteresis` (0: once failed, passes again only within the bounds narrowed by it), `trigger_cycles` (1), `escalation.count` (0: never) / `escalation.on_fault` (`error`): from that many entries failing at once, asks for that reaction instead of its `on_fault`; a non-finite value is out of bounds, an entry never received is not checked |
| `policy_framework/JumpCheck` | SafetyCheck | `prefixes`, `interfaces` → `<prefix>/<interface>`, `<interface>.max_delta` (required): fails on a change above it between two cycles (e.g. an encoder glitch) or a non-finite value; do not use on wrapping values |
| `policy_framework/MockSafetyCheck` | SafetyCheck | `topic` (`~/<name>`, `policy_framework/msg/MockFault`), `reactions` (default empty): fails while the latest message has `fault: true`, with its `reaction` (empty: `on_fault`; must be listed in `reactions`, else `error`) and for its `duration` (s, 0: until the next message); reactivation forgets the latest message. For testing how the robot reacts to a fault only, see [safety](docs/safety.rst) |
| `policy_framework/ReferenceInterfaceExecutor` | InterfaceExecutor | `prefix` (empty: the entry name), `interfaces` (`[position, velocity]`) → `<prefix>/<joint>/<interface>`; entries never written are left unchanged |
| `policy_framework/ConcatPreprocessor` | Preprocessor | `input_tensor` (`obs`), `input_entries`: entries copied as-is |
| `policy_framework/SplitPostprocessor` | Postprocessor | `output_tensor` (`actions`), `output_entries`: values written as-is |
| `policy_framework_mjlab/MjlabPreprocessor` | Preprocessor | `imu_name` (`imu`), `imu_frame` (URDF link of the IMU, for the base terms of pal_mjlab exports), `velocity_command_name` (`cmd_vel`), `odometry_name` (`odom`), `observation_input` (`obs`), `terms.<term>.type` |
| `policy_framework_mjlab/MjlabPostprocessor` | Postprocessor | `excluded_joints`, `position_interface` (`position`), `velocity_interface` (`velocity`, empty: none), `gains_controller` (empty: none), `gains_as_commands` (false) |
| `policy_framework_mjlab/TwistCommandTerm` | ObservationTerm (a `CommandTerm`) | `topic` (`cmd_vel`), `timeout` (1.0 s, above zero; on the receive time) → linear.x, linear.y, angular.z, stamped with its receive time; zero (stamped now) when absent, stale or gated; a user command |

Source and executor `topic` parameters default to `~/<name>` (also when empty).

## mjlab policies

### MjlabPreprocessor

Builds the observation from the ONNX metadata exported by mjlab (`observation_names`,
`joint_names`, `default_joint_pos`, `command_names`):

| Term | Source |
|------|--------|
| `base_ang_vel` | `<imu_name>/angular_velocity.{x,y,z}` |
| `base_lin_acc` | `<imu_name>/linear_acceleration.{x,y,z}` |
| `imu_projected_gravity` / `projected_gravity` | gravity in the frame of `<imu_name>/orientation` |
| `base_lin_vel` | `<odometry_name>/linear.{x,y,z}` |
| `joint_pos` | `<joint>/position` − `default_joint_pos` |
| `joint_vel` | `<joint>/velocity` |
| `actions` | previous raw (unscaled) `actions` output |
| `command` (`command_names: motion`) | previous `joint_pos`, `joint_vel` outputs; robot state on the first step |
| `command` (otherwise) | `<velocity_command_name>/linear.x`, `linear.y`, `angular.z` |
| any name with `terms.<name>.type` set | the `ObservationTerm` plugin of that class |

Each term is its own block of `history × dim` values, oldest first; as mjlab's
`CircularBuffer`, the first step after a clear back-fills the whole history. The history length
is `obs size / single observation size`, the same for every term. A `time_step` input receives
the steps since activation, and a motion command is re-seeded from the robot state on every
activation.

**pal_mjlab exports** are recognised by their extra metadata:

| Metadata | Effect |
|----------|--------|
| `observation_joint_names` | joints of `joint_pos` / `joint_vel` (a subset of `joint_names`) |
| `observation_terms_dim` | checked against the size of every term |
| `observation_terms_history_length` | history length of each term (0, mjlab's "no history", is read as 1: the current frame only) |
| `goal_root_body` | the base link of the policy, wherever the IMU is mounted |

| Term | Source |
|------|--------|
| `imu_ang_vel` | `<imu_name>/angular_velocity.{x,y,z}` |
| `velocity_command` | `<velocity_command_name>/linear.x`, `linear.y`, `angular.z` |
| `base_ang_vel` (with `goal_root_body`) | angular velocity of the base in its axes: the gyroscope reading moved to the base with the kinematics |
| `base_projected_gravity` | gravity in the base frame, from the IMU orientation and the kinematics |

The base terms need `kinematics.enabled` and `imu_frame` (the URDF link of
the IMU).

### MjlabPostprocessor

Writes `<joint>/position = action_scale · action + default_joint_pos` (and `<joint>/velocity =
0`) for the metadata joints minus `excluded_joints` (e.g. passive joints). pal_mjlab exports
name the actuated joints themselves (`action_joint_names`; `excluded_joints` is then not used)
and their offsets (`action_offset`, else their default positions). The policy's
`joint_stiffness`/`joint_damping` metadata gains are either
- written as `<joint>/stiffness` and `<joint>/damping` commands on every step
  (`gains_as_commands`, for `PdEffortExecutor`: they follow policy switches immediately, and
  are known before the first step so that the executor can hold the robot; missing gains are
  a configuration error), or
- sent as `gains.<joint>.p/.d` parameters of the `gains_controller` pid_controller each time
  the policy is activated, retried every 0.5 s until its parameter service is up (only the
  latest activated policy's gains when switching quickly; missing gains are logged and the
  controller's gains left untouched).

### Observation term plugins

With `<policy>.terms.<term>.type` set, a term of the model's `observation_names` is computed by
that `ObservationTerm` plugin (parameters under `<policy>.terms.<term>.`) instead of the
built-in code, which replaces a built-in term or adds one `MjlabPreprocessor` does not know.
The preprocessor keeps the history and layout; the term writes its current value, and the step
waits while it is not `ready()`. E.g. the velocity command straight from its topic:

```yaml
loco:
  preprocessor: policy_framework_mjlab/MjlabPreprocessor
  terms:
    command:
      type: policy_framework_mjlab/TwistCommandTerm
      topic: /cmd_vel
```

Topic terms need no input source and claim no state interface; keep data that exists as state
interfaces (joints, IMU) in the state store so the controller reads it in real time. A term can
also combine state entries with the kinematics, e.g. an IMU reading expressed in another link.
Fixed-size float data such as a height scan works as a term (validate its size in
`configure()`: only the total `obs` size is checked); do no non-real-time work (e.g. a TF
lookup) in `compute()` in a synchronous controller.

**Limitation: camera images are not supported as model inputs yet.** Terms only write the
single flat float32 `observation_input` (no separate `image [1,3,H,W]` input, no `uint8`), the
term-major history copied every cycle neither matches stacked image layouts nor scales to them,
and image decoding does not belong in the control loop. Planned: a per-term model input
(`terms.<term>.input`) with its own layout and history, and a `uint8` tensor type.

### Kangaroo

```bash
# simulation (arm_type:=4dof for robot_type:=full_body_4dof)
ros2 launch kangaroo_mujoco kangaroo_mujoco.launch.py arm_type:=no-arm moveit:=False
ros2 launch policy_framework_mjlab kangaroo_mjlab.launch.py mode:=controller
# walk, jump
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}"
ros2 service call /policy_controller/switch_policy policy_framework/srv/SwitchPolicy "{policy_name: imitation_v719}"
```

Several versions are loaded side by side from comma-separated lists, one policy each, named
`loco_<version>` and `imitation_<version>`. The first loco version is active at start, and every
imitation returns to it when its motion ends, whichever loco policy was active before:

```bash
ros2 launch policy_framework_mjlab kangaroo_mjlab.launch.py robot_type:=full_body_4dof \
  loco_models:=v265,v700 imitation_models:=v30,v31 imitation_frames:=300,250
ros2 service call /policy_controller/switch_policy policy_framework/srv/SwitchPolicy "{policy_name: loco_v700}"
ros2 service call /policy_controller/switch_policy policy_framework/srv/SwitchPolicy "{policy_name: imitation_v31}"
```

| `mode` | What runs |
|--------|-----------|
| `controller` (default) | `policy_controller`: policy at 50 Hz, PD law (`PdEffortExecutor`) at 1 kHz into the joint efforts, gains of the active policy (`gains_as_commands`) |
| `controller_async` | the same with `async_inference: true` |
| `node` | `policy_node` publishes references (`MultiDofCommandExecutor`) to `rl_impedance_controller` (pid_controller, 1 kHz); add `use_sim_time:=true` |

| Launch argument | Default |
|-----------------|---------|
| `robot_type` | `lower_body` (`full_body_4dof`) |
| `models_dir` | `models` of `pal_policy_deployer` when installed; layout `<models_dir>/<robot_type>/<task>/<task>_<version>_mjlab` |
| `loco_models`, `imitation_models` | comma-separated versions; default per robot type: `v315`, `v719` (lower body); `v265`, `v30` (4dof) |
| `imitation_frames` | comma-separated steps of each imitation before returning to the first loco policy (the motion's frame count); may be omitted when every motion is known: `v719` 147, `v30` 300 |
| `activate_impedance_controller` | `True`; `False` loads and configures the controllers inactive |
| `use_sim_time` | `false` (node mode only) |

The controller manager must be started with this workspace sourced to find the controller
plugin. In node mode the launch shuts down (unloading `rl_impedance_controller`) when the
policy node exits. Configuration in `policy_framework_mjlab/config/kangaroo`: `policies.yaml`
(settings per policy kind, shared by both hosts; the launch makes one policy per version),
`policy_controller.yaml`, `policy_node.yaml`,
`<robot_type>_impedance.yaml` (pid_controller, node mode).

## Writing plugins

- Derive from a base class in `policy_framework/interfaces/` (one header per kind, e.g.
  `interfaces/preprocessor.hpp`, `interfaces/observation_term.hpp`; `policy_framework/plugin_interfaces.hpp`
  includes them all except the observation terms) and export it with
  `PLUGINLIB_EXPORT_CLASS` and a description registered against `policy_framework`:
  `pluginlib_export_plugin_description_file(policy_framework plugins.xml)`.
- Parameters: generate_parameter_library with the plugin's prefix,
  `my_plugin::ParamListener(ctx.node, ctx.name).get_params()` reads `<ctx.name>.<param>`;
  `policy_framework::get_parameter<T>(ctx, "param", default)` for names only known at runtime.
- Register every state/command entry in `configure()`; in the cycle use only indices, and do
  not allocate, block or log every cycle.
- `Preprocessor::compute` and `ObservationTerm::ready` return false until their inputs arrived.
- `Preprocessor::observe` is called instead of `compute` while another policy is active: keep
  time-dependent memory (a history) current there.
- `Postprocessor::initial_commands` writes what is known before the first step (e.g. gains);
  `InterfaceExecutor::hold` computes the values that keep the robot still until then.
  `InterfaceExecutor::fault` computes the values of a fault reaction (`HOLD`, `DAMP`, `SAFE`;
  default: the safe values) and `supports_fault_reaction` says which ones the configuration
  allows (`PolicyController` refuses to configure with one an executor does not support).
- A term that feeds a user command (a velocity command, a motion reference) derives from
  `CommandTerm<Msg>` (below): it handles the `CommandGate` (`safety_entry`, the
  `NEUTRAL_COMMANDS` fault reaction), a `timeout` above zero measured on the receive time (the
  only thing that catches an operator who stopped sending), the stamp and the user command
  flag, so a command term cannot forget them. An `InputSource` feeding commands (like
  `TwistSource`) does the gate and the timeout itself.
- A `SafetyCheck` registers the state entries it reads in `configure()` and returns a `Verdict`
  from `check()`: `safe`, and when it is not, an optional `reaction`; it writes the reason only
  on failure (no allocation while it passes). Its reaction is configured per instance
  (`on_fault`); the `Verdict` may ask for another one among those `possible_reactions()` returns
  once configured. Both hosts run the safety checks on every cycle, before the policy.
- `InputSource` callbacks never touch the store: buffer the message (`LatestMessage`) and copy it
  in `read()`. An `InputSource` reports `receive_time()`, when its latest data arrived (0 before
  any), for `<source>.max_age`; `TopicSource` does it for a topic.
- Keep stamps flowing (see [Timestamps](#timestamps)). `policy_framework_mjlab` is a complete
  example.

A term fed by a custom message derives from `TopicTerm<Msg>`, which owns the subscription and
hands the latest message to the control cycle real-time safely. Its parameters are `topic`, and
`max_age` (seconds, default 0: off) with `max_age_on_fault` (`hold`, `damp`, `safe` or
`error`, default `error`): a fault when its latest message arrived longer ago, checked after
each policy step (a `CommandTerm` refuses a `max_age`: it has its `timeout`):

```cpp
using namespace policy_framework;

class MyTerm : public TopicTerm<my_msgs::msg::Foo>
{
public:
  MyTerm() : TopicTerm("foo") {}  // default topic
  double compute(const DataStore &, const TensorMap &, float * out) override
  {
    const auto in = latest();  // non-null: ready() is true once a message arrived
    out[0] = in.msg->a;  out[1] = in.msg->b;
    const auto & h = in.msg->header.stamp;   // stamp of the value: the header, else now
    return h.sec == 0 && h.nanosec == 0 ? now() : rclcpp::Time(h).seconds();
  }
protected:
  size_t configure_term(const PluginContext &, const ModelInfo &) override {return 2;}
};
```

A user command term derives from `CommandTerm<Msg>` instead, and only says how to write the
command and the neutral command; the base writes the neutral command without a message,
after its `timeout` and while the `CommandGate` is closed, stamps it, and flags it so that it
does not count for the observation stamp:

```cpp
class HeadingCommand : public CommandTerm<std_msgs::msg::Float64>
{
public:
  HeadingCommand() : CommandTerm("heading", 0.5) {}  // default topic, default timeout (s)
protected:
  size_t configure_command(const PluginContext &, const ModelInfo &) override {return 1;}
  void write_command(const std_msgs::msg::Float64 & msg, const DataStore &, float * out) override
  {
    out[0] = static_cast<float>(msg.data);
  }
  void write_neutral(const DataStore &, float * out) override {out[0] = 0.0F;}
};
```

Export it with `base_class_type="policy_framework::ObservationTerm"`. A term reading the state
store derives from `ObservationTerm`, registers its entries in `configure()` and checks them in
`ready()`.

### Using the kinematics

`ctx.kinematics` (null unless enabled; `policy_framework/core/kinematics.hpp`, also usable on its
own, and from other code through `PolicyController::kinematics()`):

| Method | |
|--------|---|
| `Kinematics(urdf, joints = {})` | Build the model |
| `joint_names()` | Order of the `q`/`v` vectors |
| `frame("pelvis")` | Link index; call while configuring (throws on unknown links) |
| `update(q, v, stamp)` | Set the joint state, compute the forward kinematics of every frame |
| `transform(f)`, `transform(f, ref)` | Pose in the root link / in another link |
| `velocity(f, LOCAL\|ROOT_ALIGNED)` | Spatial velocity (linear, angular) |
| `world_orientation(f, imu_frame, imu_quaternion)` | Orientation in the world from a measured IMU orientation (read from the state by the term) |
| `world_angular_velocity(f, imu_frame, gyro)` | Angular velocity in the world, in the frame's axes, from a gyroscope on another link |
| `ready()`, `stamp()` | An update happened; its stamp (oldest joint input) |
| `with_lock(fn)` | Several reads from the same update |

In a term: `frame()` in `configure()`, `kinematics->ready()` in `ready()`, `stamp()` as the
stamp. Writer and readers share a `realtime_tools::prio_inherit_recursive_mutex`: `update()`
never waits (it skips the update while a reader holds the lock, the next update catches up),
and readers lock for a copy only. Use `with_lock()` for several reads from the same update.
The model a term gets always matches the state its step reads, also with `async_inference`
(see [Kinematics](#kinematics)).

### Timestamps

Every value carries the time its data was produced (seconds, node clock), to measure input
ages and the latency down to the commands:

| Data | Stamp |
|------|-------|
| State entry from a topic source | message header stamp; the time it is read when there is none (e.g. Twist) |
| State entry from a state interface | time it was read (interfaces carry no stamp) |
| Observation term | `ObservationTerm::compute()` return value: the header stamp, else now; built-in terms: oldest of their entries. A user command term (`CommandTerm`) is flagged `user_command` and stamped with the header stamp, else its receive time (its age is how long ago it arrived); the neutral command is stamped now |
| Term fed back from previous outputs (`actions`, motion command; a term returns `ObservationTerm::kFromPreviousOutputs`) | previous observation stamp, flagged `from_previous_outputs` |
| Observation (`PolicyPipeline::observation_stamp()`) | oldest input stamp, feedback terms and user commands excluded |
| Command entry | observation stamp of the step that computed it |

A preprocessor reports `input_stamps()` (per term for mjlab, per entry for
`ConcatPreprocessor`), a postprocessor writes the stamp it receives on every command. The
controller publishes the age of each step's observation as introspection (see Introspection);
the node does not publish stamps (`MultiDOFCommand` has no header). Header stamps compare only
when publishers use the node's clock.

User commands do not count for the observation stamp: a command is held until its own timeout
(a joystick at 10 Hz is up to 100 ms old while driving, a motion reference may be sent once),
so it is old by design. Counted, the slowest command would make the observation look old and
hide a stale sensor behind it; `safety.max_input_age` could then not be tighter than the
slowest command. Their stamps are still reported in `input_stamps()`, and an operator who
stopped sending is caught by the command's timeout.

## Documentation

The concept pages in [`docs/`](docs/index.rst) are reStructuredText for Sphinx, with Graphviz
diagrams, rendered with the Read the Docs theme.

To build and browse them locally, run these from the repository root:

```bash
# one-time setup: Graphviz for the diagrams, Sphinx and the theme in a virtualenv
sudo apt install graphviz
python3 -m venv ~/.venvs/sphinx
~/.venvs/sphinx/bin/pip install sphinx sphinx-rtd-theme

# build the HTML pages (add -W to turn warnings into errors)
~/.venvs/sphinx/bin/sphinx-build -b html docs docs/_build/html

# serve them, then open http://localhost:8000
python3 -m http.server 8000 -d docs/_build/html
```

To rebuild the pages and reload the browser whenever a `.rst` file is saved, run this instead:

```bash
~/.venvs/sphinx/bin/pip install sphinx-autobuild
~/.venvs/sphinx/bin/sphinx-autobuild docs docs/_build/html --port 8000
```

The output in `docs/_build/` is ignored by git.

The [`docs` workflow](.github/workflows/docs.yml) builds the pages with `-W` on every push and pull
request, and publishes them to GitHub Pages from `main` (the repository's Pages source must be set
to "GitHub Actions").

## Tests

```bash
colcon build && colcon test && colcon test-result --verbose
# also run every shipped pal mjlab model through the mjlab stages:
PAL_POLICY_MODELS_DIR=<pal_policy_deployer>/share/pal_policy_deployer/models colcon test
```

The ONNX test models are generated by `policy_framework/test/fixtures/models/generate_test_models.py`.
