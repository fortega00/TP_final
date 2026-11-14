"""Launch de la etapa B: localización sobre un mapa conocido y navegación a un objetivo.

Sirve igual para el simulador y para el robot real. Lo que cambia son los
argumentos: tópico y frame de la odometría, y si este launch publica el mapa y
el TF map -> odom o ya lo hace otro.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(value):
    if value.lower() not in ('true', 'false'):
        raise ValueError(f'Expected true/false, got {value!r}')
    return value.lower() == 'true'


def _setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    files = {}
    for name in ('map_file', 'config_file'):
        if not get(name):
            raise ValueError(f'{name} is required')
        files[name] = str(Path(get(name)).expanduser().resolve())
        if not Path(files[name]).is_file():
            raise ValueError(f'{name} does not exist: {files[name]}')
    sim_time = _boolean(get('use_sim_time'))
    share = get_package_share_directory('nav')
    actions = [Node(package='nav', executable='localization_node', name='localization',
                    output='screen', parameters=[{
                        **files, 'use_sim_time': sim_time,
                        'scan_topic': get('scan_topic'), 'odom_topic': get('odom_topic'),
                        'map_frame': get('map_frame'), 'odom_frame': get('odom_frame'),
                        'base_frame': get('base_frame'),
                        'odom_child_frame': get('odom_child_frame'),
                        'publish_tf': _boolean(get('publish_tf')),
                        'publish_map': _boolean(get('publish_map')),
                    }])]
    if _boolean(get('navigate')):
        actions.append(Node(package='nav', executable='navigator_node', name='navigator',
                            output='screen', parameters=[{
                                **files, 'use_sim_time': sim_time,
                                'scan_topic': get('scan_topic'), 'cmd_vel_topic': get('cmd_vel_topic'),
                                'map_frame': get('map_frame'), 'base_frame': get('base_frame'),
                            }]))
    if _boolean(get('rviz')):
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                            arguments=['-d', os.path.join(share, 'rviz', 'nav.rviz')],
                            parameters=[{'use_sim_time': sim_time}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('nav')
    defaults = {
        'map_file': '', 'config_file': os.path.join(share, 'config', 'real.yaml'),
        'scan_topic': '/scan', 'odom_topic': '/odom',
        'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_footprint',
        'odom_child_frame': '',
        # En el robot real nadie más publica el mapa ni map -> odom. En el café
        # de custom_map ya lo hace su launch: ahí van en false.
        'publish_tf': 'true', 'publish_map': 'true',
        'use_sim_time': 'false', 'rviz': 'true',
        # navigate:=false lanza solo la localización: el robot se maneja con teleop.
        'navigate': 'true', 'cmd_vel_topic': '/cmd_vel',
    }
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [OpaqueFunction(function=_setup)])
