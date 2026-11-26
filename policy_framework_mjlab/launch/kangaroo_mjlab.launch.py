# Copyright 2026 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Deploy Kangaroo mjlab policies (locomotion + imitation) with policy_framework.

mode:=node              standalone policy_node: topics in, MultiDOFCommand reference out to
                        rl_impedance_controller (pid_controller), which closes the joint
                        impedance loop at 1 kHz
mode:=controller        policy_controller in the controller manager loop: policy at 50 Hz,
                        PD law (PdEffortExecutor) at 1 kHz into the joint effort interfaces,
                        with the gains of the active policy; no pid_controller
mode:=controller_async  the same with async_inference: true: inference in its own real-time
                        thread, the PD law stays in the 1 kHz control loop

robot_type:=full_body_4dof | lower_body selects the models and the impedance configuration.
The models are in models_dir, by default the joint-level (simple) models of pal_rl_policies;
without loco_models, full_body_4dof runs loco v1741 alone, and lower_body has no default.
use_sim_time:=true in simulation, in every mode: in the controller modes it skips the safety
checks of the real robot (config/kangaroo/safety.yaml); in node mode policy_node follows /clock
(it has no safety checks yet: no source feeds what they read).
loco_models and imitation_models take comma-separated versions, one policy each, named
loco_<version> and imitation_<version>. The first loco policy is active at start and every
imitation returns to it after its imitation_frames. Switch with:
  ros2 service call /<policy_node|policy_controller>/switch_policy \
    policy_framework_msgs/srv/SwitchPolicy "{policy_name: loco_v1740}"

config:=<file> lists the policies in a YAML file instead, with any names and per-policy
settings (a path, or relative to config/kangaroo/):
  ros2 launch policy_framework_mjlab kangaroo_mjlab.launch.py robot_type:=full_body_4dof \
    config:=examples/full_body_4dof_loco.yaml
Each policy names its `model` as <task>/<version>, under <models_dir>/<robot_type>; see that
example.
"""

import copy
import os
import tempfile

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# Robot types, each with its <robot_type>_impedance.yaml, and their default loco model in
# pal_rl_policies (None: loco_models or config must be given). No imitation runs by default.
LOCO_DEFAULTS = {
    'lower_body': None,
    'full_body_4dof': 'v1741',
}
# In the controller, the gains go to the PD law as commands instead of to
# rl_impedance_controller parameters.
CONTROLLER_OVERRIDES = {'gains_controller': '', 'gains_as_commands': True}


def default_models_dir():
    """Joint-level (simple) models shipped with pal_rl_policies, when it is installed."""
    try:
        return os.path.join(get_package_share_directory('pal_rl_policies'), 'models', 'simple')
    except PackageNotFoundError:
        return ''


def model_dir(models_dir, robot_type, task, version):
    """Resolve <models_dir>/<robot_type>/<task>/<task>_<version>_mjlab and check it exists."""
    path = os.path.join(models_dir, robot_type, task, f'{task}_{version}_mjlab')
    if not os.path.isdir(path):
        raise FileNotFoundError(f'Policy model directory not found: {path}')
    return path


def policy_parameters(template, models_dir, robot_type, loco_models, imitation_models,
                      imitation_frames, overrides=None):
    """Build the policy parameters: one policy per model version.

    :param template: settings per kind, {'loco': {...}, 'imitation': {...}}, copied into
        every policy of that kind.
    :param loco_models: loco versions; the first one is active at start and is the one every
        imitation returns to.
    :param imitation_models: imitation versions.
    :param imitation_frames: max_steps of each imitation, in the same order.
    :param overrides: settings applied to every policy after the template.
    :returns: {'policies': [names], <name>: {...}} with names loco_<v> and imitation_<v>.
    :raises ValueError: without loco models, or without one frame count per imitation.
    :raises FileNotFoundError: when a model directory does not exist.
    """
    if not loco_models:
        raise ValueError('loco_models is empty: the first loco policy is the default one')
    if len(imitation_frames) != len(imitation_models):
        raise ValueError(f'imitation_frames {imitation_frames} needs one value per '
                         f'imitation model {imitation_models} (the frame count of each motion)')

    # The same policy config as the `config` argument, so both go through one resolver.
    config = {'policies': []}
    for version in loco_models:
        config['policies'].append(f'loco_{version}')
        config[f'loco_{version}'] = {'model': f'loco/{version}'}
    for version, frames in zip(imitation_models, imitation_frames):
        config['policies'].append(f'imitation_{version}')
        config[f'imitation_{version}'] = {'model': f'imitation/{version}',
                                          'max_steps': int(frames)}
    return resolve_policies(template, models_dir, robot_type, config, overrides)


def resolve_policies(template, models_dir, robot_type, config, overrides=None):
    """Build the policy parameters from a policy config, the format of the `config` argument.

    :param template: settings per kind, {'loco': {...}, 'imitation': {...}}.
    :param config: {'policies': [names], <name>: {'model': '<task>/<version>', ...}}. Each
        policy starts from the template of its task, gets the model_path of its model under
        <models_dir>/<robot_type>, then its own settings on top. The first policy, active at
        start and the one every imitation returns to, must be a loco one. Every imitation needs
        max_steps, the frame count of its motion. The config is not changed.
    :param overrides: settings applied to every policy last.
    :returns: {'policies': [names], <name>: {...}}, the parameters of the policy host.
    :raises ValueError: when the first policy is not loco, a model is missing or not
        <task>/<version> of a known task, or an imitation has no max_steps.
    :raises FileNotFoundError: when a model directory does not exist.
    """
    names = config.get('policies', [])
    if not names:
        raise ValueError('the config has no policies: the first policy must be a loco one')
    params = {'policies': list(names)}
    for index, name in enumerate(names):
        entry = copy.deepcopy(config.get(name, {}))
        if 'model' not in entry:
            raise ValueError(f"policy '{name}' has no model (<task>/<version>)")
        parts = str(entry.pop('model')).split('/')
        if len(parts) != 2:
            raise ValueError(f"policy '{name}': model must be task/version, got {parts}")
        task, version = parts
        if task not in template:
            raise ValueError(f"policy '{name}': unknown task '{task}', one of {sorted(template)}")
        if index == 0 and task != 'loco':
            raise ValueError(f"the first policy '{name}' must be a loco one: every imitation "
                             'returns to it')
        policy = copy.deepcopy(template[task])
        policy['model_path'] = model_dir(models_dir, robot_type, task, version)
        policy.update(entry)
        if task == 'imitation' and 'max_steps' not in policy:
            raise ValueError(f"policy '{name}': set max_steps, the frame count of the motion")
        policy.update(overrides or {})
        params[name] = policy
    return params


def read_config(path):
    """Load a policy config: an absolute path, or relative to config/kangaroo/.

    :raises ValueError: when the file is empty or is a node parameter file (ros__parameters):
        the policies are given to whichever host the mode
        runs, so the config holds them alone.
    """
    path = os.path.join(get_package_share_directory('policy_framework_mjlab'), 'config',
                        'kangaroo', path)
    with open(path) as f:
        config = yaml.safe_load(f)
    if not isinstance(config, dict):
        raise ValueError(f'{path} is empty or not a mapping: it needs a policies list')
    if any(isinstance(v, dict) and 'ros__parameters' in v for v in config.values()):
        raise ValueError(f'{path} has ros__parameters: give the policies at the top level, '
                         'without a node name (see config/kangaroo/examples)')
    return config


def write_parameters(params):
    """Write parameters for any node into a temporary parameter file and return its path."""
    out = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump({'/**': {'ros__parameters': params}}, out)
    out.close()
    return out.name


def render(template, values):
    """Fill the @KEY@ placeholders of a config template into a temporary file."""
    with open(template) as f:
        text = f.read()
    for key, value in values.items():
        text = text.replace(f'@{key}@', str(value))
    out = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    out.write(text)
    out.close()
    return out.name


def safety_parameters(config, sim):
    """Parameter files of the real robot's safety checks: none in simulation.

    Only policy_controller loads them for now (#45): no source of policy_node feeds
    <joint>/absolute_position and <actuator>/mode yet.

    :param config: directory of the Kangaroo configuration.
    :param sim: True when launched against the simulation, which exposes neither
        <joint>/absolute_position nor <actuator>/mode, so the checks could not be configured.
    :returns: a list of parameter file paths, empty in simulation.
    """
    return [] if sim else [os.path.join(config, 'safety.yaml')]


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def arg_list(name):
        return [v.strip() for v in arg(name).split(',') if v.strip()]

    mode = arg('mode')
    if mode not in ('node', 'controller', 'controller_async'):
        raise ValueError(f"mode must be node, controller or controller_async, got '{mode}'")
    robot_type = arg('robot_type')
    if robot_type not in LOCO_DEFAULTS:
        raise ValueError(f'robot_type must be one of {sorted(LOCO_DEFAULTS)}, got {robot_type}')
    models_dir = arg('models_dir')
    if not models_dir:
        raise ValueError('models_dir is empty and pal_rl_policies is not installed')
    config = os.path.join(
        get_package_share_directory('policy_framework_mjlab'), 'config', 'kangaroo')
    impedance = os.path.join(config, f'{robot_type}_impedance.yaml')
    with open(os.path.join(config, 'policies.yaml')) as f:
        template = yaml.safe_load(f)
    overrides = None if mode == 'node' else CONTROLLER_OVERRIDES
    if arg('config'):
        # One source of policies: a version argument next to a config would be ignored.
        given = [n for n in ('loco_models', 'imitation_models', 'imitation_frames') if arg(n)]
        if given:
            raise ValueError(f'config replaces {given}: set one or the other')
        policies = resolve_policies(
            template, models_dir, robot_type, read_config(arg('config')), overrides)
    else:
        policies = policy_parameters(
            template, models_dir, robot_type,
            arg_list('loco_models') or [v for v in [LOCO_DEFAULTS[robot_type]] if v],
            arg_list('imitation_models'), arg_list('imitation_frames'), overrides)
    policies = write_parameters(policies)
    safety = safety_parameters(config, arg('use_sim_time') == 'true')
    # Inactive: load and configure only, e.g. to be activated later by a supervisor.
    activation = ['--unload-on-kill'] if arg('activate_impedance_controller') == 'True' \
        else ['--inactive']

    if mode == 'node':
        return [
            Node(
                package='controller_manager', executable='spawner',
                arguments=['rl_impedance_controller', '--param-file', impedance] + activation,
            ),
            Node(
                package='policy_framework', executable='policy_node', name='policy_node',
                output='screen',
                # TODO(#45): pass safety too, once a source of the node feeds
                # <joint>/absolute_position and <actuator>/mode.
                parameters=[policies, os.path.join(config, 'policy_node.yaml'),
                            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                # Without its policy the robot must not keep a stale reference: stop the
                # launch, which unloads the impedance controller (as pal_policy_deployer).
                on_exit=Shutdown(reason='policy_node exited'),
            ),
        ]

    controller = render(os.path.join(config, 'policy_controller.yaml'), {
        'ASYNC_INFERENCE': 'true' if mode == 'controller_async' else 'false'})
    return [
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['policy_controller', '--param-file', policies,
                       '--param-file', controller]
            + [a for f in safety for a in ('--param-file', f)] + activation,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='controller',
                              description='node, controller or controller_async'),
        DeclareLaunchArgument('robot_type', default_value='full_body_4dof',
                              description='full_body_4dof or lower_body'),
        DeclareLaunchArgument('models_dir', default_value=default_models_dir(),
                              description='Directory with <robot_type>/<task>/'
                                          '<task>_<model>_mjlab (default: models/simple of '
                                          'pal_rl_policies)'),
        DeclareLaunchArgument('loco_models', default_value='',
                              description='Comma-separated loco versions, the first one active '
                                          'at start (default: v1741 for full_body_4dof)'),
        DeclareLaunchArgument('imitation_models', default_value='',
                              description='Comma-separated imitation versions (default: none)'),
        DeclareLaunchArgument('imitation_frames', default_value='',
                              description='Comma-separated steps of each imitation before '
                                          'returning to the first loco (the frame count of '
                                          'each motion), required with imitation_models'),
        DeclareLaunchArgument('config', default_value='',
                              description='Policy config, a path or relative to '
                                          'config/kangaroo/ (e.g. examples/'
                                          'full_body_4dof_loco.yaml). Replaces '
                                          'loco_models, imitation_models and imitation_frames'),
        DeclareLaunchArgument('activate_impedance_controller', default_value='True',
                              choices=['True', 'False'],
                              description='False: load and configure the controllers inactive'),
        # The one simulation switch: the controllers use the manager time, but it leaves out the
        # real robot's safety checks, whose state interfaces the simulation lacks.
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                              description='true in simulation: the controllers run without the '
                                          'safety checks of the real robot '
                                          '(config/kangaroo/safety.yaml); policy_node follows '
                                          '/clock'),
        OpaqueFunction(function=launch_setup),
    ])
