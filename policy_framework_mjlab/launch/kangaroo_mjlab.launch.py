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

robot_type:=lower_body | full_body_4dof selects the models and the impedance configuration.
loco_models and imitation_models take comma-separated versions, one policy each, named
loco_<version> and imitation_<version>. The first loco policy is active at start and every
imitation returns to it after its imitation_frames. Switch with:
  ros2 service call /<policy_node|policy_controller>/switch_policy \
    policy_framework/srv/SwitchPolicy "{policy_name: imitation_v30}"

record:=<directory> publishes every policy step (publish_steps) and records it with the user
commands in an MCAP bag, to look at later or to run through a model with policy_replay.
"""

import copy
import os
import tempfile

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# Recorded with record:=<directory>, besides the policy steps of the host.
RECORDED_TOPICS = ['/cmd_vel', '/joint_states']

# Default (loco model, imitation model) per robot type, as deployed by pal_policy_deployer.
ROBOT_DEFAULTS = {
    'lower_body': ('v315', 'v719'),
    'full_body_4dof': ('v265', 'v30'),
}
# Frames of the known imitation motions (rows of 'joint_pos' in the model's motion.npz).
MOTION_FRAMES = {
    'lower_body': {'v719': 147},
    'full_body_4dof': {'v30': 300},
}
# In the controller, the gains go to the PD law as commands instead of to
# rl_impedance_controller parameters.
CONTROLLER_OVERRIDES = {'gains_controller': '', 'gains_as_commands': True}


def default_models_dir():
    """Models shipped with pal_policy_deployer, when it is installed."""
    try:
        return os.path.join(get_package_share_directory('pal_policy_deployer'), 'models')
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
    :param imitation_frames: max_steps of each imitation, in the same order; empty to use
        MOTION_FRAMES.
    :param overrides: settings applied to every policy after the template.
    :returns: {'policies': [names], <name>: {...}} with names loco_<v> and imitation_<v>.
    :raises ValueError: without loco models, or when a motion's frames are unknown.
    :raises FileNotFoundError: when a model directory does not exist.
    """
    if not loco_models:
        raise ValueError('loco_models is empty: the first loco policy is the default one')
    if imitation_frames and len(imitation_frames) != len(imitation_models):
        raise ValueError(f'imitation_frames {imitation_frames} needs one value per '
                         f'imitation model {imitation_models}')
    if not imitation_frames:
        known = MOTION_FRAMES.get(robot_type, {})
        unknown = [v for v in imitation_models if v not in known]
        if unknown:
            raise ValueError(f'set imitation_frames for the motions {unknown} (one value per '
                             'imitation model, the frame count of each motion)')
        imitation_frames = [known[v] for v in imitation_models]

    params = {'policies': []}

    def add(task, version, extra):
        name = f'{task}_{version}'
        policy = copy.deepcopy(template[task])
        policy['model_path'] = model_dir(models_dir, robot_type, task, version)
        policy.update(extra)
        policy.update(overrides or {})
        params['policies'].append(name)
        params[name] = policy

    for version in loco_models:
        add('loco', version, {})
    for version, frames in zip(imitation_models, imitation_frames):
        add('imitation', version, {'max_steps': int(frames)})
    return params


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


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def arg_list(name):
        return [v.strip() for v in arg(name).split(',') if v.strip()]

    mode = arg('mode')
    if mode not in ('node', 'controller', 'controller_async'):
        raise ValueError(f"mode must be node, controller or controller_async, got '{mode}'")
    robot_type = arg('robot_type')
    if robot_type not in ROBOT_DEFAULTS:
        raise ValueError(f'robot_type must be one of {sorted(ROBOT_DEFAULTS)}, got {robot_type}')
    loco, imitation = ROBOT_DEFAULTS[robot_type]
    models_dir = arg('models_dir')
    if not models_dir:
        raise ValueError('models_dir is empty and pal_policy_deployer is not installed')
    config = os.path.join(
        get_package_share_directory('policy_framework_mjlab'), 'config', 'kangaroo')
    impedance = os.path.join(config, f'{robot_type}_impedance.yaml')
    with open(os.path.join(config, 'policies.yaml')) as f:
        template = yaml.safe_load(f)
    policies = write_parameters(policy_parameters(
        template, models_dir, robot_type,
        arg_list('loco_models') or [loco],
        arg_list('imitation_models') or [imitation],
        arg_list('imitation_frames'),
        overrides=None if mode == 'node' else CONTROLLER_OVERRIDES))
    host = 'policy_node' if mode == 'node' else 'policy_controller'
    record = arg('record')
    recording = [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--storage', 'mcap', '--output', record,
                 f'/{host}/policy_step'] + RECORDED_TOPICS,
            output='screen'),
    ] if record else []
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
                parameters=[policies, os.path.join(config, 'policy_node.yaml'),
                            {'use_sim_time': LaunchConfiguration('use_sim_time'),
                             'publish_steps': bool(record)}],
                # Without its policy the robot must not keep a stale reference: stop the
                # launch, which unloads the impedance controller (as pal_policy_deployer).
                on_exit=Shutdown(reason='policy_node exited'),
            ),
        ] + recording

    controller = render(os.path.join(config, 'policy_controller.yaml'), {
        'ASYNC_INFERENCE': 'true' if mode == 'controller_async' else 'false',
        'PUBLISH_STEPS': 'true' if record else 'false'})
    return [
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['policy_controller', '--param-file', policies,
                       '--param-file', controller] + activation,
        ),
    ] + recording


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='controller',
                              description='node, controller or controller_async'),
        DeclareLaunchArgument('robot_type', default_value='lower_body',
                              description='lower_body or full_body_4dof'),
        DeclareLaunchArgument('models_dir', default_value=default_models_dir(),
                              description='Directory with <robot_type>/<task>/<task>_<model>_mjlab'),
        DeclareLaunchArgument('loco_models', default_value='',
                              description='Comma-separated loco versions, the first one active '
                                          'at start (default per robot_type)'),
        DeclareLaunchArgument('imitation_models', default_value='',
                              description='Comma-separated imitation versions '
                                          '(default per robot_type)'),
        DeclareLaunchArgument('imitation_frames', default_value='',
                              description='Comma-separated steps of each imitation before '
                                          'returning to the first loco (known motions: default)'),
        DeclareLaunchArgument('activate_impedance_controller', default_value='True',
                              choices=['True', 'False'],
                              description='False: load and configure the controllers inactive'),
        DeclareLaunchArgument('record', default_value='',
                              description='Directory of an MCAP bag recording every policy step '
                                          'and the user commands; empty: no recording'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Node mode only; controllers use the manager time'),
        OpaqueFunction(function=launch_setup),
    ])
