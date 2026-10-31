# ros2_policy_framework

Run learned policies (e.g. mjlab-trained ONNX policies) on ROS 2 robots, with a clear split
between **where data comes from**, **what the policy does with it** and **where commands go**.
The same policy configuration runs

- as a **standalone node** (topics in, topics out), or
- as a **ros2_control controller**, synchronously in the control loop or asynchronously
  (`is_async: true`), reading state interfaces and/or topics and writing command interfaces
  and/or topics,

and several policies can be loaded at once and **switched at runtime**.

## Packages

| Package | Content |
|---------|---------|
| `policy_framework` | Core: plugin base classes (incl. `ObservationTerm`, `TopicTerm`), `DataStore`, `PolicyPipeline`, `PolicyRunner`, `PolicyNode` (standalone), `PolicyController` (ros2_control), generic plugins (ONNX inference, topic sources, MultiDOFCommand executor, PD effort and reference interface executors, concat/split stages), `SwitchPolicy` service |
| `policy_framework_mjlab` | Pre/postprocessor plugins for mjlab policies (observation terms with history, action scaling, per-policy PID gains), the `TwistCommandTerm` observation term and the Kangaroo deployment config/launch |

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

- **DataStore**: flat table of named scalars. Names follow the ros2_control convention
  `<prefix>/<interface>` (`leg_left_1_joint/position`, `torso_imu_sensor/orientation.x`,
  `cmd_vel/linear.x`). Topic sources write exactly the names the equivalent state interfaces
  have, so moving an input from a topic to a state interface (or back) is configuration only.
  Entries are registered at configure time; the control cycle only uses indices.
- **Preprocessor** reads the state store and fills the model inputs. It also receives the
  model outputs of the previous step (to feed back the last action, an autoregressive motion
  reference, ...), and owns the observation history. `MjlabPreprocessor` can be extended
  with **`ObservationTerm` plugins**, which read the state store or subscribe to a topic
  directly (data with no state interface equivalent: velocity commands, custom messages).
- **Inference** runs the model (`policy_framework/OnnxInference`: ONNX Runtime, CPU,
  zero-copy binding to the pipeline buffers). Tensors are named float32 arrays
  (`TensorMap`), so the other stages do not depend on the inference backend.
- **Postprocessor** turns model outputs into named commands (`<joint>/position`, ...).
- **Executors** send the commands. In the standalone node they are `Executor` plugins that
  publish topics. In the controller they are `InterfaceExecutor` plugins that compute command
  interface values, e.g. a joint PD law into efforts (`PdEffortExecutor`) or a copy into the
  reference interfaces of a chained controller (`ReferenceInterfaceExecutor`). Each host
  rejects the other kind.

One cycle: sources → [state interfaces] → preprocessor → inference → postprocessor →
executors. A preprocessor returning "not ready" (an input never
received) skips the cycle without sending anything.

### Timestamps

Every value carries the time its data was produced (seconds, node clock), so the age of each
input and the latency down to the commands can be measured:

| Data | Stamp |
|------|-------|
| State entry from a topic source | message header stamp; receive time when the header is empty or absent (Twist) |
| State entry from a state interface | cycle time when read (ros2_control interfaces carry no stamp) |
| Observation term | `ObservationTerm::compute()` return value; built-in mjlab terms: oldest stamp of their entries |
| Term fed back from previous outputs (`actions`, motion command) | stamp of the previous observation, flagged `from_previous_outputs` |
| Observation (`PolicyPipeline::observation_stamp()`) | oldest known input stamp, feedback terms excluded |
| Command entry | the observation stamp of the step that computed it |

`Preprocessor::input_stamps()` lists one stamp per input (per term for mjlab, per entry for
`ConcatPreprocessor`), and `PolicyPipeline::input_stamps()` returns the active policy's.
Stamps are not published yet: this is the basis for adding timing/latency statistics later.
Header stamps are only comparable when the publisher uses the same clock as the node (e.g.
sim time everywhere). `MultiDOFCommand` has no header, so the executor does not forward the
stamp.

### Policy switching

`policies` is a list; the first one is active at start. Switch with

```bash
ros2 service call /<node or controller>/switch_policy policy_framework/srv/SwitchPolicy "{policy_name: imitation}"
```

The switch is applied at the start of the next cycle and calls `reset()` on the activated
policy's stages (e.g. restart a motion's time step). A policy keeps its memory (observation
history, previous outputs) when switched away and back, as `pal_policy_deployer` does; it is
cleared (`Preprocessor::clear()`, outputs zeroed) only before its first activation and when
the node/controller is (re)activated. Per policy:

- `max_steps` (default 0 = unlimited): after N steps return to the first policy (e.g. the
  length of a jump motion);
- `interruptible` (default true): when false, switch requests are rejected while it runs.

All policies are loaded (models included) at configuration, so switching never allocates or
loads anything in the control loop.

### Kinematics

A runner can hold one kinematic model of a chain of the robot, built from the robot
description with a [kinematics_interface](https://github.com/ros-controls/kinematics_interface)
plugin (`kinematics_interface_kdl`). It is
updated once at the start of every policy step from the state store, and every policy plugin
(and observation term) reads it through `PluginContext::kinematics`, so terms never compute
or update kinematics themselves:

```yaml
kinematics:
  plugin_name: kinematics_interface_kdl/KinematicsInterfaceKDL
  base: base_link            # transforms are expressed in it
  tip: torso_imu_link        # base must be an ancestor of tip
  imu_sensor: torso_imu_sensor   # optional: <imu_sensor>/orientation.* (IMU orientation in the world)
  imu_link: torso_imu_link       # ... and its link, on the chain: enables world_orientation()
```

It registers `<joint>/position` of the chain's movable joints (and the IMU orientation) as
state entries: the controller claims them as state interfaces, the node needs sources for
them. The controller uses the controller manager's robot description; the node reads its
`robot_description` parameter.

In a plugin: `frame("pelvis")` in `configure()` (throws if the link is not on the chain), then
`transform(f)` (pose in `base`) or `world_orientation(f)` during the step; check
`kinematics->ready()` (all joints and IMU received) in `ready()` and use `stamp()` (oldest
input) as the stamp. `ctx.kinematics` is null when `kinematics.plugin_name` is empty.

The wrapper fills the plugin's joint vector with the chain's movable joints from `base` to
`tip`, the KDL plugin's convention. Another plugin (e.g. `kinematics_interface_pinocchio`) must
be checked to use the same joint vector before switching `plugin_name`.

Limits: one chain per runner (the KDL plugin models a single base→tip chain, and only its
links are available); orientation only, no base position or linear velocity (that needs a
state estimator); it runs in the policy step thread (with `async_inference`, on the inference
thread's copy of the state), never in interface executors.

## Running

### Standalone node

```bash
ros2 run policy_framework policy_node --ros-args --params-file my_policies.yaml
```

A lifecycle node that configures and activates itself (`autostart: true`, default). All
policy inputs must come from input sources; configuration fails listing the inputs no source
provides. Commands leave through topic executors (e.g. `MultiDofCommandExecutor`). On a crash
it prints a stack trace (`backward_ros`; the controller manager's `ros2_control_node` links it
too). `update_rate` (Hz, default 50) is the policy rate; the timer follows `/clock` with
`use_sim_time`.

### ros2_control controller

```yaml
controller_manager:
  ros__parameters:
    policy_controller:
      type: policy_framework/PolicyController

policy_controller:
  ros__parameters:
    update_rate: 1000        # controller rate: the executors (PD law) run on every update
    policy_rate: 50.0        # policy step every update_rate / policy_rate updates (0: every update)
    async_inference: true    # inference in its own RT thread, the PD law stays in this loop
    sources: []              # topic sources; anything else needed is claimed as state interface
    executors: [pd]          # InterfaceExecutor plugins
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

- **Inputs**: every state entry a policy or an executor needs that no input source provides is
  claimed as the state interface of the same name. Topics only, state interfaces only or a mix
  all work for policy inputs; the feedback of an executor (e.g. PD joint positions/velocities)
  must come from state interfaces, read on every update (configuration fails otherwise).
- **Outputs**: each executor claims its command interfaces (`PdEffortExecutor`:
  `<joint>/effort`; `ReferenceInterfaceExecutor`: `<prefix>/<entry>`). Two executors cannot
  claim the same interface. Topic executors are rejected: they only run in the node.
- **Rates**: with `policy_rate` the policy steps every `update_rate / policy_rate` updates
  (must be an integer) and the executors run on every update, e.g. a 1 kHz PD law with a
  50 Hz policy. Without it, one policy step per update: `update_rate` is the policy rate.
- **Async inference** (`async_inference: true`, recommended with a PD law): only the policy
  step runs in its own real-time thread (`SCHED_FIFO`, `inference_thread_priority`, default
  40, below the controller manager's 50). On each policy tick `update()` hands over a copy of
  the state and triggers the step without waiting; the commands apply from the first update
  after the step finished, the previous ones are held until then, and the executors keep
  running on every update with the fresh state. A tick that finds the previous step still
  running is skipped with a warning (inference slower than `policy_rate`). Without real-time
  permissions the thread runs with the default scheduler (a warning is logged).
- **is_async**: the standard ros2_control option moves the whole `update()` (inference and
  executors) to a separate thread at `update_rate`. Suited to a reference executor (the
  chained controller keeps the loop closed), not to a PD law. Exclusive with
  `async_inference`.
- **Safety**: executors run only once the active policy produced commands since activation.
  On deactivation and when a stage fails (`update()` then returns `ERROR`) each executor
  writes its safe values: zero effort for `PdEffortExecutor`, nothing for
  `ReferenceInterfaceExecutor` (the chained controller holds the last reference). A policy
  still waiting for inputs returns `OK` and sends nothing.
- On activation the first policy restarts from scratch.

### Parameters

All parameters are declared and validated with
[generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library):
the definitions (types, defaults, descriptions, validation) are in `params/*.yaml` of each
package. Every parameter is read at configure time and is read-only. Invalid values (e.g. a
negative `max_steps`, `num_threads: 0`, an empty `policies`) make configuration fail with
the parameter name. Write floating point parameters with a decimal point (`50.0`): in the
controller they are declared from the YAML value first.

| Parameter | Description |
|-----------|-------------|
| `sources` | Input source names; each needs `<source>.type` |
| `policies` | Policy names (required); the first one is the default |
| `<policy>.preprocessor` / `.postprocessor` | Plugin classes (required) |
| `<policy>.inference` | Plugin class, default `policy_framework/OnnxInference` |
| `<policy>.max_steps`, `<policy>.interruptible` | Switching behaviour, see above |
| `<policy>.<plugin param>` | Parameters of the policy's plugins |
| `executors` | Executor names; each needs `<executor>.type` (`Executor` in the node, `InterfaceExecutor` in the controller) |
| `policy_rate` | Controller only: policy rate in Hz (default 0 = every update) |
| `async_inference`, `inference_thread_priority` | Controller only: policy step in its own real-time thread (default false), its `SCHED_FIFO` priority (default 40) |
| `kinematics.plugin_name`, `.base`, `.tip`, `.alpha`, `.imu_sensor`, `.imu_link` | Optional kinematic model, see [Kinematics](#kinematics) (empty `plugin_name`, default: none) |
| `robot_description` | Node only: URDF for the kinematics (the controller uses the controller manager's) |

### Plugins

| Plugin | Type | Parameters (under `<name>.`) |
|--------|------|------------------------------|
| `policy_framework/OnnxInference` | Inference | `model_path` (file, or directory: first `.onnx`), `num_threads` (1) |
| `policy_framework/JointStateSource` | InputSource | `topic`, `joints` (empty = any) → `<joint>/position\|velocity\|effort` |
| `policy_framework/ImuSource` | InputSource | `topic`, `sensor_name` → `<sensor>/orientation.*`, `angular_velocity.*`, `linear_acceleration.*` |
| `policy_framework/TwistSource` | InputSource | `topic`, `timeout` (1.0 s) → `<name>/linear.*`, `angular.*`; zero when absent or stale |
| `policy_framework/OdometrySource` | InputSource | `topic` → `<name>/position.*`, `orientation.*`, `linear.*`, `angular.*` |
| `policy_framework/MultiDofCommandExecutor` | Executor (node) | `topic`, `values_interface` (`position`), `values_dot_interface` (`velocity`) |
| `policy_framework/PdEffortExecutor` | InterfaceExecutor (controller) | `joints` (default: every `<joint>/position` command), `effort_interface` (`effort`), `stiffness`/`damping` (per joint, used when the policy does not write `<joint>/stiffness`/`damping` commands), `effort_limits` (per joint, empty = none) |
| `policy_framework/ReferenceInterfaceExecutor` | InterfaceExecutor (controller) | `prefix`, `interfaces` (`[position, velocity]`) → `<prefix>/<joint>/<interface>` |
| `policy_framework/ConcatPreprocessor` | Preprocessor | `input_tensor` (`obs`), `input_entries`: entries copied as-is |
| `policy_framework/SplitPostprocessor` | Postprocessor | `output_tensor` (`actions`), `output_entries`: values written as-is |
| `policy_framework_mjlab/MjlabPreprocessor` | Preprocessor | `imu_name` (`imu`), `velocity_command_name` (`cmd_vel`), `odometry_name` (`odom`), `observation_input` (`obs`), `terms.<term>.type` (term plugin, see below) |
| `policy_framework_mjlab/MjlabPostprocessor` | Postprocessor | `excluded_joints`, `position_interface`, `velocity_interface`, `gains_controller`, `gains_as_commands` (false) |
| `policy_framework_mjlab/TwistCommandTerm` | ObservationTerm | `topic` (`cmd_vel`), `timeout` (1.0 s) → linear.x, linear.y, angular.z of a Twist; zero when absent or stale |

Source and executor `topic` parameters default to `~/<name>` (an empty value also means
that); `ImuSource.sensor_name` defaults to the source name.

## mjlab policies

`MjlabPreprocessor` builds the observation from the ONNX metadata exported by mjlab
(`observation_names`, `joint_names`, `default_joint_pos`, `command_names`):

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
`history × dim` values, oldest first; as in mjlab's `CircularBuffer`, the first step after a
clear back-fills the whole history with the current value. A `time_step` model input
receives the number of steps since activation, and a motion command is re-seeded from the
robot state on every activation.

#### Observation term plugins

A term listed in the model's `observation_names` is computed by an `ObservationTerm` plugin
instead of the built-in code when `<policy>.terms.<term>.type` is set; its parameters are
under `<policy>.terms.<term>.`. This replaces a built-in term or adds a term
`MjlabPreprocessor` does not know, without modifying it. The preprocessor still keeps the
history and the layout; the term only writes its current value, and the cycle waits while a
term is not `ready()`. For example, the velocity command straight from its topic:

```yaml
loco:
  preprocessor: policy_framework_mjlab/MjlabPreprocessor
  terms:
    command:
      type: policy_framework_mjlab/TwistCommandTerm
      topic: /cmd_vel
      timeout: 1.0
```

Topic terms need no input source and claim no state interface. Keep data that also exists as
state interfaces (joints, IMU) in the state store, so the controller reads it in real time.

Fixed-size float data such as a height scan works as a term: the term returns its size from
parameters (only the total `obs` size is checked against the model, so validate it in
`configure_term()`) and copies the values in `compute()`. Do not do non-real-time work in
`compute()` (e.g. a TF lookup) in a synchronous controller: publish the sampled scan from
another node, or use `is_async: true`.

**Limitation: camera images are not supported as model inputs yet.**
- Terms only write into the single flat `observation_input` tensor; a separate image input
  (e.g. `image [1,3,H,W]`) cannot be filled by `MjlabPreprocessor`.
- Tensors are float32 only, so models taking `uint8` images cannot be fed.
- Term history is term-major and shifted by copy every cycle, which neither matches stacked
  image layouts (`[T,C,H,W]`) nor scales to image sizes.
- Image decoding/resizing does not belong in the synchronous control loop.

Planned options: let a term target its own model input (`<policy>.terms.<term>.input`) with
its own layout and optional history, and add a tensor dtype when a `uint8` model is needed.

`MjlabPostprocessor` writes `<joint>/position = action_scale · action + default_joint_pos`
(and `<joint>/velocity = 0`) for the metadata joints minus `excluded_joints`, and with
`gains_controller` pushes the policy's `joint_stiffness`/`joint_damping` as
`gains.<joint>.p/.d` parameters of that pid_controller each time the policy is activated
(only the latest activated policy's gains when switching quickly). With `gains_as_commands`
it writes them as `<joint>/stiffness` and `<joint>/damping` commands on every step instead, for
`PdEffortExecutor`: the gains then follow policy switches immediately.

### Kangaroo

```bash
# simulation (arm_type:=4dof for robot_type:=full_body_4dof)
ros2 launch kangaroo_mujoco kangaroo_mujoco.launch.py arm_type:=no-arm moveit:=False
# policies: mode:=controller (default) | controller_async | node (add use_sim_time:=true)
ros2 launch policy_framework_mjlab kangaroo_mjlab.launch.py mode:=controller
# walk, jump
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}"
ros2 service call /policy_controller/switch_policy policy_framework/srv/SwitchPolicy "{policy_name: imitation}"
```

| Launch argument | Default |
|-----------------|---------|
| `mode` | `controller` (`controller_async`, `node`) |
| `robot_type` | `lower_body` (`full_body_4dof`) |
| `models_dir` | `models` of `pal_policy_deployer` when installed |
| `loco_model`, `imitation_model`, `imitation_frames` | per robot type: `v315`, `v719`, 147 (lower body); `v265`, `v30`, 300 (4dof) |
| `activate_impedance_controller` | `True`; `False` loads and configures the controllers inactive |
| `use_sim_time` | `false` (node mode only) |

- **Node mode**: `policy_node` publishes the reference (`MultiDofCommandExecutor`) to
  `rl_impedance_controller` (pid_controller), which closes the joint impedance loop at 1 kHz.
- **Controller modes**: `policy_controller` runs the policy at 50 Hz and the PD law
  (`PdEffortExecutor`) at 1 kHz into the joint effort interfaces, with the gains of the active
  policy (`gains_as_commands`); no pid_controller. `controller_async` sets
  `async_inference: true`: inference in its own real-time thread, the PD law in the 1 kHz
  loop.

The controller manager must be started with this workspace sourced so that it finds the
controller plugin. In node mode the launch shuts down (unloading `rl_impedance_controller`)
when the policy node exits.

Configuration files in `policy_framework_mjlab/config`: `kangaroo_policies.yaml` (policies,
shared by node and controller), `kangaroo_policy_controller.yaml` (PD effort controller),
`kangaroo_policy_node.yaml`, `kangaroo_<robot_type>_impedance.yaml` (pid_controller, node
mode).

## Writing plugins

Derive from a base class of `policy_framework/plugin_interfaces.hpp` (or `ObservationTerm` /
`TopicTerm` of `policy_framework/observation_term.hpp`), export it with
`PLUGINLIB_EXPORT_CLASS` and a plugin description registered against `policy_framework`:

```cmake
pluginlib_export_plugin_description_file(policy_framework plugins.xml)
```

Declare plugin parameters with generate_parameter_library, instantiating the generated
listener with the plugin's prefix: `my_plugin::ParamListener(ctx.node, ctx.name).get_params()`
reads `<ctx.name>.<param>` (parameters are declared once, so several policies can share a
plugin type). `policy_framework::get_parameter<T>(ctx, "param", default)` remains for
parameters whose names are only known at runtime (e.g. `terms.<term>.type`). Keep stamps flowing: a preprocessor reports `input_stamps()`, a
postprocessor writes the `stamp` it receives on every command, a term returns the stamp of
its value. Register every state/command entry in `configure()`; in the cycle
only use indices. `Preprocessor::compute` returns false until its inputs are received.
`InputSource` callbacks must not touch the store: buffer the message (e.g.
`realtime_tools::RealtimeBuffer`) and copy it in `read()`. `policy_framework_mjlab` is a
complete example.

A new observation term (e.g. from a custom message) derives from
`policy_framework::TopicTerm<Msg>` (`policy_framework/observation_term.hpp`), which owns the
subscription and hands the latest message to the control cycle real-time safely:

```cpp
class MyTerm : public policy_framework::TopicTerm<my_msgs::msg::Foo>
{
public:
  MyTerm() : TopicTerm("foo") {}  // default topic
  double compute(const DataStore &, const TensorMap &, float * out) override
  {
    const auto in = latest();  // msg non-null: ready() is true once a message arrived
    out[0] = in.msg->a;  out[1] = in.msg->b;
    return rclcpp::Time(in.msg->header.stamp).seconds();  // stamp of the value
  }
protected:
  size_t configure_term(const PluginContext & ctx, const ModelInfo &) override {return 2;}
};
```

Export it with `base_class_type="policy_framework::ObservationTerm"`. A term reading the
state store derives from `ObservationTerm` directly, registers its entries in `configure()`
and checks them in `ready()`.

## Tests

```bash
colcon build && colcon test && colcon test-result --verbose
# also run every shipped pal mjlab model through the mjlab stages:
PAL_POLICY_MODELS_DIR=<pal_policy_deployer>/share/pal_policy_deployer/models colcon test
```

The ONNX test model is generated by `policy_framework/test/models/generate_test_models.py`.
