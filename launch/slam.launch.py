"""One entry point for toy, external Gazebo simulation, ROS bag and real robot."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(value):
    if value.lower() not in ('true', 'false'):
        raise ValueError(f'Expected true/false, got {value!r}')
    return value.lower() == 'true'


def _setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    mode = get('mode')
    if mode not in ('toy', 'simulation', 'bag', 'real'):
        raise ValueError('mode must be toy, simulation, bag or real')
    config_file = str(Path(get('config_file')).expanduser().resolve())
    if not Path(config_file).is_file():
        raise ValueError(f'Configuration file does not exist: {config_file}')
    share = get_package_share_directory('tp_slam')
    sim_time = mode != 'real' if get('use_sim_time') == 'auto' else _boolean(get('use_sim_time'))
    parameters = {
        'config_file': config_file, 'use_sim_time': sim_time,
        'mode': mode, 'scan_topic': get('scan_topic'), 'odom_topic': get('odom_topic'),
        'odom_source': get('odom_source'), 'map_frame': get('map_frame'),
        'odom_frame': get('odom_frame'), 'base_frame': get('base_frame'),
        'publish_odom_tf': _boolean(get('publish_odom_tf')),
        'output_path': str(Path(get('output_path')).expanduser().resolve()),
        'publish_period': float(get('publish_period')),
        'tf_wait_timeout': float(get('tf_wait_timeout')),
        'odom_history_seconds': float(get('odom_history_seconds')),
        'scan_queue_size': int(get('scan_queue_size')),
        'max_path_poses': int(get('max_path_poses')),
        'ground_truth_topic': (('/toy/ground_truth' if mode == 'toy' else
                                '/ground_truth/odom' if mode == 'simulation' else '')
                               if get('ground_truth_topic') == 'auto' else get('ground_truth_topic')),
    }
    actions = [Node(package='tp_slam', executable='slam_node', name='slam',
                    parameters=[parameters], output='screen')]
    if mode == 'toy':
        if _boolean(get('publish_scan_tf')):
            raise ValueError('Toy publishes its own identity sensor TF; publish_scan_tf must be false')
        actions.append(Node(package='tp_slam', executable='toy_node', output='screen', parameters=[{
            'config_file': config_file, 'use_sim_time': sim_time, 'rate': float(get('toy_rate')),
            'scan_topic': get('scan_topic'), 'odom_topic': get('odom_topic'),
            'odom_frame': get('odom_frame'), 'base_frame': get('base_frame'),
            'laser_frame': get('laser_frame'),
        }]))
    if mode == 'simulation' and _boolean(get('start_simulator')):
        world = str(Path(get('world')).expanduser().resolve())
        if not Path(world).is_file():
            raise ValueError(f'World does not exist: {world}')
        gazebo_share = get_package_share_directory('yahboom_rosmaster_gazebo')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            gazebo_share, 'launch', 'rosmaster_gazebo_fortress.launch.py')),
            launch_arguments={'world': world, 'use_sim_time': str(sim_time).lower(),
                              'rviz': 'false', 'headless': get('headless'),
                              'motion_bias': 'false', 'motion_profile': 'ideal'}.items()))
    if mode == 'bag':
        bag_path = Path(get('bag_path')).expanduser().resolve()
        if not bag_path.exists() or not get('bag_path'):
            raise ValueError('mode:=bag requires an existing bag_path')
        rate = float(get('bag_rate'))
        if rate <= 0:
            raise ValueError('bag_rate must be positive')
        actions.append(Node(package='tp_slam', executable='bag_tf_filter', output='screen',
                            parameters=[{'use_sim_time': sim_time, 'map_frame': get('map_frame'),
                                         'odom_frame': get('odom_frame')}]))
        command = ['ros2', 'bag', 'play', str(bag_path), '--clock', '--rate', str(rate),
                   '--qos-profile-overrides-path', os.path.join(share, 'config', 'bag_qos.yaml'),
                   '--topics', get('scan_topic'), get('odom_topic'), '/tf', '/tf_static',
                   '--remap', '/tf:=/slam/bag_tf', '/tf_static:=/slam/bag_tf_static']
        actions.append(TimerAction(period=3.0, actions=[ExecuteProcess(cmd=command, output='screen')]))
    if mode != 'toy' and _boolean(get('publish_scan_tf')):
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name='explicit_laser_mount', output='screen',
                            parameters=[{'use_sim_time': sim_time}], arguments=[
                                '--x', get('laser_x'), '--y', get('laser_y'), '--z', get('laser_z'),
                                '--yaw', get('laser_yaw'), '--pitch', '0', '--roll', '0',
                                '--frame-id', get('base_frame'), '--child-frame-id', get('laser_frame')]))
    if _boolean(get('rviz')):
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                            arguments=['-d', os.path.join(share, 'rviz', 'slam.rviz')],
                            parameters=[{'use_sim_time': sim_time}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('tp_slam')
    defaults = {
        'mode': 'toy', 'config_file': os.path.join(share, 'config', 'default.yaml'),
        'scan_topic': '/scan', 'odom_topic': '/odom', 'odom_source': 'topic',
        'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_footprint',
        'output_path': './maps/slam', 'rviz': 'true', 'use_sim_time': 'auto',
        'bag_path': '', 'bag_rate': '0.5', 'publish_odom_tf': 'false',
        'world': os.path.join(share, 'worlds', 'office.sdf'), 'start_simulator': 'true',
        'headless': 'false', 'toy_rate': '10.0', 'publish_scan_tf': 'false',
        'laser_frame': 'laser_frame', 'laser_x': '0.0', 'laser_y': '0.0',
        'laser_z': '0.0', 'laser_yaw': '0.0',
        'publish_period': '1.0', 'tf_wait_timeout': '2.0',
        'odom_history_seconds': '30.0', 'scan_queue_size': '30', 'max_path_poses': '5000',
        'ground_truth_topic': 'auto',
    }
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [OpaqueFunction(function=_setup)])
