# Copyright 2026 PAL Robotics S.L.
# SPDX-License-Identifier: Apache-2.0
"""Deploy the Kangaroo lower body mjlab policies (loco + imitation) with policy_framework.

mode:=node              standalone policy_node: topics in, MultiDOFCommand reference out
mode:=controller        policy_controller in the controller manager loop (synchronous)
mode:=controller_async  policy_controller with is_async: true (own thread)

In every mode rl_impedance_controller (pid_controller) closes the joint impedance loop at
1 kHz; in the controller modes the policy is chained into its reference interfaces.
Switch to the imitation policy with:
  ros2 service call /<policy_node|policy_controller>/switch_policy \
    policy_framework/srv/SwitchPolicy "{policy_name: imitation}"
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_models_dir():
    """Models shipped with pal_policy_deployer, when it is installed."""
    try:
        return os.path.join(get_package_share_directory('pal_policy_deployer'), 'models')
    except PackageNotFoundError:
        return ''


def model_dir(models_dir, task, version):
    """Resolve <models_dir>/lower_body/<task>/<task>_<version>_mjlab and check it exists."""
    path = os.path.join(models_dir, 'lower_body', task, f'{task}_{version}_mjlab')
    if not os.path.isdir(path):
        raise FileNotFoundError(f'Policy model directory not found: {path}')
    return path


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

    mode = arg('mode')
    if mode not in ('node', 'controller', 'controller_async'):
        raise ValueError(f"mode must be node, controller or controller_async, got '{mode}'")
    models_dir = arg('models_dir')
    if not models_dir:
        raise ValueError('models_dir is empty and pal_policy_deployer is not installed')
    config = os.path.join(get_package_share_directory('policy_framework_mjlab'), 'config')
    impedance = os.path.join(config, 'kangaroo_lower_body_impedance.yaml')
    policies = render(os.path.join(config, 'kangaroo_lower_body_policies.yaml'), {
        'LOCO_MODEL_DIR': model_dir(models_dir, 'loco', arg('loco_model')),
        'IMITATION_MODEL_DIR': model_dir(models_dir, 'imitation', arg('imitation_model')),
        'IMITATION_FRAMES': int(arg('imitation_frames')),
    })

    if mode == 'node':
        return [
            Node(
                package='controller_manager', executable='spawner',
                arguments=['rl_impedance_controller', '--param-file', impedance,
                           '--unload-on-kill'],
            ),
            Node(
                package='policy_framework', executable='policy_node', name='policy_node',
                output='screen',
                parameters=[policies, os.path.join(config, 'kangaroo_policy_node.yaml'),
                            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            ),
        ]

    controller = render(os.path.join(config, 'kangaroo_policy_controller.yaml'), {
        'IS_ASYNC': 'true' if mode == 'controller_async' else 'false'})
    return [
        Node(
            package='controller_manager', executable='spawner', output='screen',
            # Activated together: policy_controller claims the pid reference interfaces.
            arguments=['rl_impedance_controller', 'policy_controller',
                       '--param-file', impedance, '--param-file', policies,
                       '--param-file', controller, '--activate-as-group', '--unload-on-kill'],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='controller',
                              description='node, controller or controller_async'),
        DeclareLaunchArgument('models_dir', default_value=default_models_dir(),
                              description='Directory with <robot>/<task>/<task>_<model>_mjlab'),
        DeclareLaunchArgument('loco_model', default_value='v315',
                              description='Version of the default locomotion policy'),
        DeclareLaunchArgument('imitation_model', default_value='v719',
                              description='Version of the imitation policy'),
        DeclareLaunchArgument('imitation_frames', default_value='147',
                              description='Steps of the imitation policy before returning'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Node mode only; controllers use the manager time'),
        OpaqueFunction(function=launch_setup),
    ])
