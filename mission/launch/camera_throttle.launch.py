"""Republica la cámara a baja frecuencia (ver mission/ros/camera_throttle_node.py).

Ejemplo: ros2 launch mission camera_throttle.launch.py rate_hz:=3.0
"""
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
    return [Node(package='mission', executable='camera_throttle_node', name='camera_throttle',
                 output='screen', parameters=[{
                     'use_sim_time': _boolean(get('use_sim_time')),
                     'rate_hz': float(get('rate_hz')),
                     'pair_tolerance': float(get('pair_tolerance')),
                     'use_depth': _boolean(get('use_depth')),
                     'color_topic': get('color_topic'), 'color_info_topic': get('color_info_topic'),
                     'depth_topic': get('depth_topic'), 'depth_info_topic': get('depth_info_topic'),
                     'output_prefix': get('output_prefix'),
                 }])]


def generate_launch_description():
    defaults = {
        'rate_hz': '3.0', 'pair_tolerance': '0.05', 'use_depth': 'true',
        'color_topic': '/cam_1/color/image_raw', 'color_info_topic': '/cam_1/color/camera_info',
        'depth_topic': '/cam_1/depth/image_raw', 'depth_info_topic': '/cam_1/depth/camera_info',
        'output_prefix': '/mission/cam', 'use_sim_time': 'false',
    }
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [OpaqueFunction(function=_setup)])
