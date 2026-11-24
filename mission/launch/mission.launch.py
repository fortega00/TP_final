"""Misión de la etapa C, y opcionalmente la navegación de la etapa B con el mismo mapa.

Ejemplo, robot real:
    ros2 launch mission mission.launch.py map_file:=/ruta/map.yaml

Con `with_nav:=false` se lanza solo la misión, para usar una navegación ya lanzada.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(value):
    if value.lower() not in ('true', 'false'):
        raise ValueError(f'Expected true/false, got {value!r}')
    return value.lower() == 'true'


def _setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    files = {}
    for name in ('map_file', 'config_file', 'nav_config_file'):
        if not get(name):
            raise ValueError(f'{name} is required')
        files[name] = str(Path(get(name)).expanduser().resolve())
        if not Path(files[name]).is_file():
            raise ValueError(f'{name} does not exist: {files[name]}')
    sim_time = _boolean(get('use_sim_time'))
    actions = [Node(package='mission', executable='mission_node', name='mission', output='screen',
                    parameters=[{**files, 'use_sim_time': sim_time, 'map_frame': get('map_frame'),
                                 'color_topic': get('color_topic'), 'depth_topic': get('depth_topic'),
                                 'camera_info_topic': get('camera_info_topic'),
                                 'camera_rate': float(get('camera_rate'))}])]
    if _boolean(get('with_nav')):
        nav_launch = os.path.join(get_package_share_directory('nav'), 'launch', 'nav.launch.py')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(nav_launch), launch_arguments={
            'map_file': files['map_file'], 'config_file': files['nav_config_file'],
            'base_frame': get('base_frame'), 'odom_topic': get('odom_topic'),
            'use_sim_time': get('use_sim_time'), 'rviz': get('rviz'),
        }.items()))
    return actions


def generate_launch_description():
    defaults = {
        'map_file': '',
        'config_file': os.path.join(get_package_share_directory('mission'), 'config', 'real.yaml'),
        'nav_config_file': os.path.join(get_package_share_directory('nav'), 'config', 'real.yaml'),
        'map_frame': 'map', 'use_sim_time': 'false',
        'color_topic': '/cam_1/color/image_raw', 'depth_topic': '/cam_1/depth/image_raw',
        'camera_info_topic': '/cam_1/color/camera_info', 'camera_rate': '5.0',
        # Navegación de la etapa B en el mismo launch.
        'with_nav': 'true', 'base_frame': 'base_footprint', 'odom_topic': '/odom', 'rviz': 'true',
    }
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [OpaqueFunction(function=_setup)])
