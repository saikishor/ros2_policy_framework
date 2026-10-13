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
| `policy_framework` | Core: plugin base classes (incl. `ObservationTerm`, `TopicTerm`), `DataStore`, `PolicyPipeline`, `PolicyRunner`, `PolicyNode` (standalone), `PolicyController` (ros2_control), generic plugins (ONNX inference, topic sources, MultiDOFCommand executor, concat/split stages), `SwitchPolicy` service |
| `policy_framework_mjlab` | Pre/postprocessor plugins for mjlab policies (observation terms with history, action scaling, per-policy PID gains), the `TwistCommandTerm` observation term and the Kangaroo deployment config/launch |

## Architecture

```
                 shared by all policies                 one per policy                 shared
 ┌──────────────────────────────────┐   ┌────────────────────────────────────────┐   ┌─────────────────────────┐
 │ InputSource plugins (topics)  ─┐ │   │ Preprocessor → Inference → Postprocessor│   │ command interfaces      │
 │ state interfaces (controller) ─┴─┼──▶│   state ──▶ TensorMap ──▶ TensorMap ──▶  ├──▶│  (controller)           │
 │            state DataStore       │   │                             commands   │   │ Executor plugins (topics)│
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
- **Executors / command interfaces** send the commands.

One cycle: sources → [state interfaces] → preprocessor → inference → postprocessor →
[command interfaces] → executors. A preprocessor returning "not ready" (an input never
received) skips the cycle without sending anything.

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

## Running

### Standalone node

```bash
ros2 run policy_framework policy_node --ros-args --params-file my_policies.yaml
```

A lifecycle node that configures and activates itself (`autostart: true`, default). All
policy inputs must come from input sources; configuration fails listing the inputs no source
provides. Commands leave through executors. `update_rate` (Hz, default 50) is the policy rate;
the timer follows `/clock` with `use_sim_time`.

### ros2_control controller

```yaml
controller_manager:
  ros__parameters:
    policy_controller:
      type: policy_framework/PolicyController

policy_controller:
  ros__parameters:
    update_rate: 50          # = policy rate: one policy step per update
    is_async: false          # true: update() (inference) runs in its own thread
    command_interface_prefix: rl_impedance_controller   # chain into a pid_controller
    use_command_interfaces: true
    sources: []              # topic sources; anything else the policies need is claimed as state interface
    # ... policies as below
```

- **Inputs**: every state entry a policy needs that no input source provides is claimed as
  the state interface of the same name. Topics only, state interfaces only or a mix all work.
- **Outputs**: with `use_command_interfaces` every command entry `e` is claimed as command
  interface `<command_interface_prefix>/e` (no prefix when empty). Executors can be added to
  publish as well, or instead (`use_command_interfaces: false`). Only entries a policy wrote
  are sent.
- **Sync / async**: standard ros2_control `is_async`. In async mode the controller manager
  runs `update()` in a separate thread at `update_rate`; the command interfaces hold the last
  command in between.
- On activation the first policy restarts from scratch. A failing stage (e.g. inference)
  makes `update()` return `ERROR`; a policy still waiting for inputs returns `OK` and
  sends nothing.

### Parameters

| Parameter | Description |
|-----------|-------------|
| `sources` | Input source names; each needs `<source>.type` |
| `policies` | Policy names (required); the first one is the default |
| `<policy>.preprocessor` / `.postprocessor` | Plugin classes (required) |
| `<policy>.inference` | Plugin class, default `policy_framework/OnnxInference` |
| `<policy>.max_steps`, `<policy>.interruptible` | Switching behaviour, see above |
| `<policy>.<plugin param>` | Parameters of the policy's plugins |
| `executors` | Executor names; each needs `<executor>.type` |

### Plugins

| Plugin | Type | Parameters (under `<name>.`) |
|--------|------|------------------------------|
| `policy_framework/OnnxInference` | Inference | `model_path` (file, or directory: first `.onnx`), `num_threads` (1) |
| `policy_framework/JointStateSource` | InputSource | `topic`, `joints` (empty = any) → `<joint>/position\|velocity\|effort` |
| `policy_framework/ImuSource` | InputSource | `topic`, `sensor_name` → `<sensor>/orientation.*`, `angular_velocity.*`, `linear_acceleration.*` |
| `policy_framework/TwistSource` | InputSource | `topic`, `timeout` (1.0 s) → `<name>/linear.*`, `angular.*`; zero when absent or stale |
| `policy_framework/OdometrySource` | InputSource | `topic` → `<name>/position.*`, `orientation.*`, `linear.*`, `angular.*` |
| `policy_framework/MultiDofCommandExecutor` | Executor | `topic`, `values_interface` (`position`), `values_dot_interface` (`velocity`) |
| `policy_framework/ConcatPreprocessor` | Preprocessor | `input_tensor` (`obs`), `input_entries`: entries copied as-is |
| `policy_framework/SplitPostprocessor` | Postprocessor | `output_tensor` (`actions`), `output_entries`: values written as-is |
| `policy_framework_mjlab/MjlabPreprocessor` | Preprocessor | `imu_name` (`imu`), `velocity_command_name` (`cmd_vel`), `odometry_name` (`odom`), `observation_input` (`obs`), `terms.<term>.type` (term plugin, see below) |
| `policy_framework_mjlab/MjlabPostprocessor` | Postprocessor | `excluded_joints`, `position_interface`, `velocity_interface`, `gains_controller` |
| `policy_framework_mjlab/TwistCommandTerm` | ObservationTerm | `topic` (`cmd_vel`), `timeout` (1.0 s) → linear.x, linear.y, angular.z of a Twist; zero when absent or stale |

Source and executor `topic` parameters default to `~/<name>`.

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

`MjlabPostprocessor` writes `<joint>/position = action_scale · action + default_joint_pos`
(and `<joint>/velocity = 0`) for the metadata joints minus `excluded_joints`, and with
`gains_controller` pushes the policy's `joint_stiffness`/`joint_damping` as
`gains.<joint>.p/.d` parameters of that pid_controller each time the policy is activated
(only the latest activated policy's gains when switching quickly).

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

The controller manager must be started with this workspace sourced so that it finds the
controller plugin. In node mode the launch shuts down (unloading `rl_impedance_controller`)
when the policy node exits.

Configuration files in `policy_framework_mjlab/config`: `kangaroo_policies.yaml` (policies,
shared by node and controller), `kangaroo_policy_controller.yaml`, `kangaroo_policy_node.yaml`,
`kangaroo_<robot_type>_impedance.yaml` (pid_controller).

## Writing plugins

Derive from a base class of `policy_framework/plugin_interfaces.hpp` (or `ObservationTerm` /
`TopicTerm` of `policy_framework/observation_term.hpp`), export it with
`PLUGINLIB_EXPORT_CLASS` and a plugin description registered against `policy_framework`:

```cmake
pluginlib_export_plugin_description_file(policy_framework plugins.xml)
```

Read parameters with `policy_framework::get_parameter<T>(ctx, "param", default)` (declares
`<ctx.name>.param` once). Register every state/command entry in `configure()`; in the cycle
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
  void compute(const DataStore &, const TensorMap &, float * out) override
  {
    const auto msg = latest().msg;  // non-null: ready() is true once a message arrived
    out[0] = msg->a;  out[1] = msg->b;
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
