# ros2_policy_framework

Run learned policies (e.g. mjlab-trained ONNX policies) on ROS 2 robots, with a clear split
between **where data comes from**, **what the policy does with it** and **where commands go**.
The same policy configuration runs

- as a **standalone node**: topics in, topics out; or
- as a **ros2_control controller**: state interfaces and/or topics in, command interfaces out,
  in the control loop, with inference in its own thread (`async_inference`) or fully
  asynchronous (`is_async`),

and several policies can be loaded at once and **switched at runtime**.

## Packages

| Package | Content |
|---------|---------|
| `policy_framework` | Core: plugin base classes (incl. `ObservationTerm`, `TopicTerm`), `DataStore`, `PolicyPipeline`, `PolicyRunner`, `Kinematics`, `PolicyNode` (standalone), `PolicyController` (ros2_control), generic plugins (ONNX inference, topic sources, MultiDOFCommand executor, PD effort and reference interface executors, concat/split stages), `SwitchPolicy` service |
| `policy_framework_mjlab` | Pre/postprocessor plugins for mjlab policies (observation terms with history, action scaling, per-policy gains), the `TwistCommandTerm` observation term and the Kangaroo deployment config/launch |

## Architecture

```
                 shared by all policies                 one per policy                 shared
 ┌──────────────────────────────────┐   ┌────────────────────────────────────────┐   ┌─────────────────────────┐
 │ InputSource plugins (topics)  ─┐ │   │ Preprocessor → Inference → Postprocessor│   │ node: Executor plugins  │
 │ state interfaces (controller) ─┴─┼──▶│   state ──▶ TensorMap ──▶ TensorMap ──▶  ├──▶│  (topics)               │
 │            state DataStore       │   │                             commands   │   │ controller: Interface-  │
 │                                  │   │                                        │   │  Executor plugins       │
 └──────────────────────────────────┘   └────────────────────────────────────────┘   └─────────────────────────┘
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

One cycle: [controller: state interfaces] → sources → kinematics → preprocessor → inference →
postprocessor → executors. While an input was never received the preprocessor skips the step:
the node sends nothing, the controller keeps the last commands (see Safety).

## Running

### Standalone node

```bash
ros2 run policy_framework policy_node --ros-args --params-file my_policies.yaml
```

A lifecycle node that configures and activates itself (`autostart`). Every policy input must
come from an input source (configuration fails listing the missing ones); commands leave
through topic executors. The policy steps on a timer at `update_rate`, which follows `/clock`
with `use_sim_time`. Crashes print a stack trace (`backward_ros`).

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
  that finds the step still running is skipped with a warning. Without real-time permissions
  the thread uses the default scheduler.
- **is_async**: the standard ros2_control option moves the whole `update()` to its own thread.
  Suits a reference executor, not a PD law. Exclusive with `async_inference`.
- **Safety**: executors run only once the active policy produced commands since activation;
  after that a step waiting for data keeps the last commands. On deactivation and when a stage
  fails (`update()` returns `ERROR`) each executor writes its safe values: zero effort for
  `PdEffortExecutor`, nothing for `ReferenceInterfaceExecutor` (the chained controller holds its
  reference).

## Configuration

### Parameters

All parameters are declared and validated with
[generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library)
(definitions in `params/*.yaml` of each package), read at configure time and read-only, except
the on-demand `<policy>.terms.<term>.type`. Invalid values fail configuration with the parameter
name. Write floating point values with a decimal point (`50.0`).

| Parameter | Description |
|-----------|-------------|
| `sources` | Input source names; each needs `<source>.type` |
| `policies` | Policy names (required); the first one is the default |
| `<policy>.preprocessor` / `.postprocessor` | Plugin classes (required) |
| `<policy>.inference` | Plugin class, default `policy_framework/OnnxInference` |
| `<policy>.max_steps`, `.interruptible` | Switching, see below |
| `<policy>.<plugin param>` | Parameters of the policy's plugins |
| `executors` | Executor names; each needs `<executor>.type` (`Executor` in the node, `InterfaceExecutor` in the controller) |
| `kinematics.enabled`, `kinematics.joints` | Whole-body kinematics (default false), see below |
| `policy_rate` | Controller: policy rate in Hz (default 0 = every update) |
| `async_inference`, `inference_thread_priority` | Controller: policy step in its own thread (default false), its priority (default 40) |
| `update_rate` | Node: policy rate in Hz (default 50) |
| `autostart` | Node: configure and activate on startup (default true) |
| `robot_description` | Node: URDF for the kinematics (the controller uses the controller manager's) |

### Policy switching

```bash
ros2 service call /<node or controller>/switch_policy policy_framework/srv/SwitchPolicy "{policy_name: imitation}"
```

A switch applies at the start of the next step and calls `reset()` on the activated policy's
stages; requesting the active policy restarts it. A policy keeps its memory (history, previous
outputs) when switched away and back, as `pal_policy_deployer` does; it is cleared only before
its first activation and on every (re)activation of the node/controller, which also restarts
the first policy. Per policy, `max_steps` (default 0 = unlimited) returns to the first policy
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
Limits: no base position or linear velocity in the world (needs a state estimator); joints
must be revolute, continuous or prismatic (lock the others).

### Plugins

| Plugin | Type | Parameters (under `<name>.`) |
|--------|------|------------------------------|
| `policy_framework/OnnxInference` | Inference | `model_path` (file, or directory: its first `.onnx` by name), `num_threads` (1); float32 tensors, dynamic dimensions fixed to 1 |
| `policy_framework/JointStateSource` | InputSource | `topic`, `joints` (empty = any) → `<joint>/position\|velocity\|effort` |
| `policy_framework/ImuSource` | InputSource | `topic`, `sensor_name` (source name) → `<sensor>/orientation.*`, `angular_velocity.*`, `linear_acceleration.*` |
| `policy_framework/TwistSource` | InputSource | `topic`, `timeout` (1.0 s, ≤ 0 disables) → `<name>/linear.*`, `angular.*`; zero when absent or stale |
| `policy_framework/OdometrySource` | InputSource | `topic` → `<name>/position.*`, `orientation.*`, `linear.*`, `angular.*` |
| `policy_framework/MultiDofCommandExecutor` | Executor (node) | `topic`, `values_interface` (`position`), `values_dot_interface` (`velocity`, empty: 0); drops a message while the previous one is being published |
| `policy_framework/PdEffortExecutor` | InterfaceExecutor | `joints` (default: every `<joint>/position` command), `effort_interface` (`effort`), `stiffness`/`damping` (per joint, used when the policy writes no `<joint>/stiffness`/`damping` commands), `effort_limits` (per joint, 0 or empty: none); a joint with a missing or non-finite value is left unchanged |
| `policy_framework/ReferenceInterfaceExecutor` | InterfaceExecutor | `prefix` (empty: the entry name), `interfaces` (`[position, velocity]`) → `<prefix>/<joint>/<interface>`; entries never written are left unchanged |
| `policy_framework/ConcatPreprocessor` | Preprocessor | `input_tensor` (`obs`), `input_entries`: entries copied as-is |
| `policy_framework/SplitPostprocessor` | Postprocessor | `output_tensor` (`actions`), `output_entries`: values written as-is |
| `policy_framework_mjlab/MjlabPreprocessor` | Preprocessor | `imu_name` (`imu`), `velocity_command_name` (`cmd_vel`), `odometry_name` (`odom`), `observation_input` (`obs`), `terms.<term>.type` |
| `policy_framework_mjlab/MjlabPostprocessor` | Postprocessor | `excluded_joints`, `position_interface` (`position`), `velocity_interface` (`velocity`, empty: none), `gains_controller` (empty: none), `gains_as_commands` (false) |
| `policy_framework_mjlab/TwistCommandTerm` | ObservationTerm | `topic` (`cmd_vel`), `timeout` (1.0 s, ≤ 0 disables) → linear.x, linear.y, angular.z; zero when absent or stale |

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

The history length is `obs size / single observation size`. Each term is its own block of
`history × dim` values, oldest first; as mjlab's `CircularBuffer`, the first step after a clear
back-fills the whole history. A `time_step` input receives the steps since activation, and a
motion command is re-seeded from the robot state on every activation.

### MjlabPostprocessor

Writes `<joint>/position = action_scale · action + default_joint_pos` (and `<joint>/velocity =
0`) for the metadata joints minus `excluded_joints` (e.g. passive joints). The policy's
`joint_stiffness`/`joint_damping` metadata gains are either
- written as `<joint>/stiffness` and `<joint>/damping` commands on every step
  (`gains_as_commands`, for `PdEffortExecutor`: they follow policy switches immediately;
  missing gains are a configuration error), or
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
ros2 service call /policy_controller/switch_policy policy_framework/srv/SwitchPolicy "{policy_name: imitation}"
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
| `loco_model`, `imitation_model`, `imitation_frames` | versions and motion length per robot type: `v315`, `v719`, 147 (lower body); `v265`, `v30`, 300 (4dof) |
| `activate_impedance_controller` | `True`; `False` loads and configures the controllers inactive |
| `use_sim_time` | `false` (node mode only) |

The controller manager must be started with this workspace sourced to find the controller
plugin. In node mode the launch shuts down (unloading `rl_impedance_controller`) when the
policy node exits. Configuration in `policy_framework_mjlab/config`: `kangaroo_policies.yaml`
(policies, shared by both hosts), `kangaroo_policy_controller.yaml`, `kangaroo_policy_node.yaml`,
`kangaroo_<robot_type>_impedance.yaml` (pid_controller, node mode).

## Writing plugins

- Derive from a base class of `policy_framework/plugin_interfaces.hpp` (or `ObservationTerm` /
  `TopicTerm` of `policy_framework/observation_term.hpp`) and export it with
  `PLUGINLIB_EXPORT_CLASS` and a description registered against `policy_framework`:
  `pluginlib_export_plugin_description_file(policy_framework plugins.xml)`.
- Parameters: generate_parameter_library with the plugin's prefix,
  `my_plugin::ParamListener(ctx.node, ctx.name).get_params()` reads `<ctx.name>.<param>`;
  `policy_framework::get_parameter<T>(ctx, "param", default)` for names only known at runtime.
- Register every state/command entry in `configure()`; in the cycle use only indices, and do
  not allocate, block or log every cycle.
- `Preprocessor::compute` and `ObservationTerm::ready` return false until their inputs arrived.
- `InputSource` callbacks never touch the store: buffer the message (`LatestMessage`) and copy it
  in `read()`.
- Keep stamps flowing (see [Timestamps](#timestamps)). `policy_framework_mjlab` is a complete
  example.

A term fed by a custom message derives from `TopicTerm<Msg>`, which owns the subscription and
hands the latest message to the control cycle real-time safely:

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
    return rclcpp::Time(in.msg->header.stamp).seconds();  // stamp of the value
  }
protected:
  size_t configure_term(const PluginContext &, const ModelInfo &) override {return 2;}
};
```

Export it with `base_class_type="policy_framework::ObservationTerm"`. A term reading the state
store derives from `ObservationTerm`, registers its entries in `configure()` and checks them in
`ready()`.

### Using the kinematics

`ctx.kinematics` (null unless enabled; `policy_framework/kinematics.hpp`, also usable on its
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
| `ready()`, `stamp()` | An update happened; its stamp (oldest joint input) |
| `with_lock(fn)` | Several reads from the same update |

In a term: `frame()` in `configure()`, `kinematics->ready()` in `ready()`, `stamp()` as the
stamp. Writer and readers share a `realtime_tools::prio_inherit_recursive_mutex`: `update()`
never waits (it skips the update while a reader holds the lock, the next update catches up),
and readers lock for a copy only. The model is not part of the state snapshot of
`async_inference`, so use `with_lock()` for consistent reads there.

### Timestamps

Every value carries the time its data was produced (seconds, node clock), to measure input
ages and the latency down to the commands:

| Data | Stamp |
|------|-------|
| State entry from a topic source | message header stamp; receive time when there is none (e.g. Twist) |
| State entry from a state interface | time it was read (interfaces carry no stamp) |
| Observation term | `ObservationTerm::compute()` return value; built-in terms: oldest of their entries |
| Term fed back from previous outputs (`actions`, motion command) | previous observation stamp, flagged `from_previous_outputs` |
| Observation (`PolicyPipeline::observation_stamp()`) | oldest input stamp, feedback terms excluded |
| Command entry | observation stamp of the step that computed it |

A preprocessor reports `input_stamps()` (per term for mjlab, per entry for
`ConcatPreprocessor`), a postprocessor writes the stamp it receives on every command. Stamps are
not published yet (`MultiDOFCommand` has no header). Header stamps compare only when publishers
use the node's clock.

## Tests

```bash
colcon build && colcon test && colcon test-result --verbose
# also run every shipped pal mjlab model through the mjlab stages:
PAL_POLICY_MODELS_DIR=<pal_policy_deployer>/share/pal_policy_deployer/models colcon test
```

The ONNX test models are generated by `policy_framework/test/models/generate_test_models.py`.
