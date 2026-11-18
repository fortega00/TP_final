#!/usr/bin/env python3
"""Run one ROS experiment, export its actual map and capture its RViz window.

The wall follower is an experimental Gazebo driver, independent of FastSLAM.
Ground truth is observed only for evaluation, never for driving or estimation.
Run inside the prepared Humble container; see A.md.
"""
import argparse
from contextlib import suppress
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import numpy as np
import yaml
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

from slam.core.geometry import Pose2D, wrap_angle
from slam.ros.ros_node import yaw_from_quaternion


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def path_rows(message):
    return [[seconds(p.header.stamp), p.pose.position.x, p.pose.position.y,
             yaw_from_quaternion(p.pose.orientation)] for p in message.poses]


class LeftWallFollower:
    """Explicit approach/follow/turn/corner states using body-frame laser points."""
    def __init__(self):
        self.state = 'APPROACH'
        self.target_yaw = None
        self.transitions = []
        self.clearance = 0.70

    def transition(self, state, stamp):
        if state != self.state:
            self.transitions.append({'t': stamp, 'from': self.state, 'to': state})
            self.state = state
            self.state_started = stamp

    def command(self, points, yaw, stamp, position=None):
        x, y = points[:, 0], points[:, 1]
        front = x[(x > 0) & (np.abs(y) < 0.32)]
        front = float(front.min()) if len(front) else 12.0
        # A close obstacle behind the chassis must not reverse an in-place
        # turn. Front clearance below governs translation; overriding the
        # turn by the side of a rear return causes angular oscillation.
        if self.state == 'APPROACH':
            if front > 0.80:
                return min(0.25, 0.4 * (front - 0.55)), 0.0
            self.target_yaw = wrap_angle(yaw - math.pi / 2)
            self.transition('TURN_RIGHT', stamp)
        if self.state == 'TURN_RIGHT':
            error = wrap_angle(self.target_yaw - yaw)
            if abs(error) > 0.05:
                return 0.0, float(np.clip(1.4 * error, -0.40, 0.40))
            self.transition('FOLLOW_LEFT', stamp)
        if front < 0.65:
            self.target_yaw = wrap_angle(yaw - math.pi / 2)
            self.transition('TURN_RIGHT', stamp)
            return 0.0, -0.35
        # Use a local line fit rather than a single side-facing beam. All input
        # points come from /scan transformed with the actual laser mount.
        left = points[(y > 0.15) & (y < 1.8) & (x > -0.6) & (x < 0.8)]
        if len(left) < 5:
            if self.state == 'CORNER_LEFT' and stamp - self.state_started > 4.5:
                # After about a quarter turn, seek the next wall instead of
                # orbiting forever in an open patch of floor.
                self.transition('APPROACH', stamp)
                return 0.20, 0.0
            self.transition('CORNER_LEFT', stamp)
            return 0.12, 0.32
        center = np.mean(left, axis=0)
        _, vectors = np.linalg.eigh(np.cov(left.T))
        tangent = vectors[:, -1]
        if tangent[0] < 0:
            tangent = -tangent
        angle = math.atan2(tangent[1], tangent[0])
        normal = np.array([-tangent[1], tangent[0]])
        distance = abs(float(center @ normal))
        turn = 1.4 * angle + 1.2 * (distance - self.clearance)
        self.transition('FOLLOW_LEFT', stamp)
        turn = float(np.clip(turn, -0.40, 0.40))
        speed = 0.25 * max(0.30, 1.0 - abs(turn) / 0.55)
        speed = min(speed, max(0.0, (front - 0.45) * 0.5))
        return speed, turn


class GuidedRoute:
    """Predeclared test waypoints tracked using odometry, without ground truth."""
    def __init__(self, path):
        self.waypoints = yaml.safe_load(path.read_text())['waypoints']
        self.index, self.state, self.transitions = 0, 'ALIGN', []
        self.done = False

    def command(self, points, yaw, stamp, position=None):
        if self.done or position is None:
            return 0.0, 0.0
        target = np.asarray(self.waypoints[self.index])
        delta = target - position
        distance = float(np.linalg.norm(delta))
        if distance < .08:
            self.transitions.append({'t': stamp, 'reached_waypoint': self.index})
            self.index += 1
            if self.index == len(self.waypoints):
                self.state, self.done = 'DONE', True
                return 0.0, 0.0
            self.state = 'ALIGN'
            return 0.0, 0.0
        error = wrap_angle(math.atan2(delta[1], delta[0])-yaw)
        if abs(error) > .10 and self.state != 'DRIVE':
            return 0.0, float(np.clip(1.4*error, -.4, .4))
        self.state = 'DRIVE'
        front = points[(points[:, 0] > 0) & (np.abs(points[:, 1]) < .27), 0]
        clearance = float(front.min()) if len(front) else 12.0
        if clearance < .40:
            self.state = 'BLOCKED'
            return 0.0, 0.0
        if abs(error) > .5:
            self.state = 'ALIGN'
            return 0.0, float(np.clip(1.4*error, -.4, .4))
        speed = min(.25, .7*distance, max(0, (clearance-.35)*.5))
        return speed, float(np.clip(1.8*error, -.4, .4))


class Observer(Node):
    def __init__(self, drive, route=None):
        super().__init__('entrega_a_experiment')
        persistent = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.paths, self.status, self.status_history = {}, {}, []
        self.scans = []
        self.last_scan_wall = None
        self.first_stamp = self.last_stamp = None
        self.buffer = Buffer(cache_time=Duration(seconds=120))
        self.listener = TransformListener(self.buffer, self)
        self.follower = (GuidedRoute(route) if route else LeftWallFollower()) if drive else None
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) if drive else None
        self.create_subscription(String, '/slam/status', self.on_status, persistent)
        self.create_subscription(LaserScan, '/scan', self.on_scan, sensor_qos)
        for key, topic in [('slam', '/path'), ('odom', '/slam/odom_path'),
                           ('truth', '/slam/ground_truth_path')]:
            self.create_subscription(PathMsg, topic,
                                     lambda msg, k=key: self.paths.__setitem__(k, path_rows(msg)),
                                     persistent)
        self.create_timer(0.1, self.drive)
        self.stop_driving = bool(drive)  # Arm only after a stationary startup reset.
        self.latest_points = self.latest_yaw = self.latest_position = None
        self.commands = []

    def on_status(self, message):
        self.status = json.loads(message.data)
        self.status_history.append({'wall': time.monotonic(), **self.status})

    def on_scan(self, msg):
        stamp = seconds(msg.header.stamp)
        self.last_stamp = stamp
        self.first_stamp = stamp if self.first_stamp is None else self.first_stamp
        self.last_scan_wall = time.monotonic()
        # Retain enough sensor data for honest, reproducible consistency metrics.
        self.scans.append((stamp, msg.angle_min, msg.angle_increment,
                           msg.range_min, msg.range_max, np.asarray(msg.ranges, dtype=np.float32)))
        if self.follower is None:
            return
        try:
            mount = self.buffer.lookup_transform('base_footprint', msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp)).transform
            base = self.buffer.lookup_transform('odom', 'base_footprint',
                                                Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            self.latest_points = None
            return
        values = np.asarray(msg.ranges)
        angles = msg.angle_min + np.arange(len(values)) * msg.angle_increment
        # The simulated chassis returns rays at 0.14--0.20 m behind the laser.
        # Ignore the robot's own footprint, consistently with cafe.yaml.
        valid = np.isfinite(values) & (values > max(msg.range_min, 0.25)) & (values < msg.range_max)
        angles = angles[valid] + yaw_from_quaternion(mount.rotation)
        self.latest_points = np.column_stack((mount.translation.x + values[valid] * np.cos(angles),
                                               mount.translation.y + values[valid] * np.sin(angles)))
        self.latest_yaw = yaw_from_quaternion(base.rotation)
        self.latest_position = np.array([base.translation.x, base.translation.y])

    def drive(self):
        if self.publisher is None:
            return
        command = Twist()
        if (not self.stop_driving and self.latest_points is not None
                and self.last_scan_wall is not None
                and time.monotonic() - self.last_scan_wall < 2.0):
            v, w = self.follower.command(self.latest_points, self.latest_yaw,
                                         self.last_stamp, self.latest_position)
            command.linear.x, command.angular.z = float(v), float(w)
            self.commands.append([self.last_stamp, float(v), float(w), self.follower.state])
        self.publisher.publish(command)


def bag_info(path):
    metadata = yaml.safe_load((path / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    connection = sqlite3.connect('file:' + str(path / metadata['relative_file_paths'][0]) + '?mode=ro', uri=True)
    topic = connection.execute('select id from topics where name="/scan"').fetchone()[0]
    last = connection.execute('select data from messages where topic_id=? order by timestamp desc limit 1',
                              (topic,)).fetchone()[0]
    stamp = seconds(deserialize_message(last, LaserScan).header.stamp)
    connection.close()
    return {'duration_s': metadata['duration']['nanoseconds'] / 1e9,
            'last_scan_stamp': stamp, 'topics': {t['topic_metadata']['name']: t['message_count']
                                                for t in metadata['topics_with_message_count']}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['simulation', 'bag'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--bag', type=Path, default=Path('/tmp/real_rosbag_input'))
    parser.add_argument('--bag-rate', type=float, default=0.15)
    parser.add_argument('--max-sim-seconds', type=float, default=600)
    parser.add_argument('--timeout', type=float, default=3600)
    parser.add_argument('--display', default=':93')
    parser.add_argument('--route', type=Path, help='Predeclared odometry-guided waypoints instead of wall following')
    parser.add_argument('--domain', type=int, default=93)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                      IGN_PARTITION='slam_entrega_a_' + str(args.domain),
                      DISPLAY=args.display, QT_QPA_PLATFORM='xcb', LIBGL_ALWAYS_SOFTWARE='1')
    os.environ.pop('XAUTHORITY', None)
    processes, logs = [], []
    result = {'mode': args.mode, 'configuration': yaml.safe_load(args.config.read_text()),
              'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), 'success': False}
    (output / 'executor.py').write_bytes(Path(__file__).read_bytes())
    result['environment'] = {key: os.environ.get(key) for key in
                             ('ROS_DOMAIN_ID', 'IGN_PARTITION', 'DISPLAY', 'ROS_DISTRO',
                              'LIBGL_ALWAYS_SOFTWARE')}
    if args.route:
        (output / 'route.yaml').write_bytes(args.route.read_bytes())
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    def launch(name, command):
        log = (output / (name + '.log')).open('w')
        logs.append(log)
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(child)
        result.setdefault('processes', []).append({'name': name, 'pid': child.pid, 'command': command})
        (output / 'processes.json').write_text(json.dumps(result['processes'], indent=2)+'\n')
        return child

    def pump(duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.03)

    node = app = None
    started = time.monotonic()
    try:
        virtual = launch('xvfb', ['Xvfb', args.display, '-screen', '0', '1440x1080x24', '-ac', '-nolisten', 'tcp'])
        time.sleep(0.5)
        if virtual.poll() is not None:
            raise RuntimeError('Virtual display failed; use an unused display number')
        from PyQt5.QtWidgets import QApplication
        app = QApplication([])
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = Observer(args.mode == 'simulation', args.route)
        command = ['ros2', 'launch', 'slam', 'slam.launch.py', 'rviz:=false',
                   'mode:=' + args.mode, 'config_file:=' + str(args.config.resolve()),
                   'scan_queue_size:=200',
                   'tf_wait_timeout:=' + ('60' if args.mode == 'bag' else '10'),
                   'max_path_poses:=20000',
                   'output_path:=' + str(output / 'map')]
        if args.mode == 'simulation':
            cafe = Path(get_package_share_directory('custom_map'))
            for key in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'):
                os.environ[key] = str(cafe / 'models') + os.pathsep + os.environ.get(key, '')
            command += ['headless:=true', 'world:=' + str(cafe / 'worlds/cafe.world')]
        else:
            result['bag'] = bag_info(args.bag)
            command += ['bag_path:=' + str(args.bag), 'bag_rate:=' + str(args.bag_rate)]
        result['launch_command'] = command
        slam = launch('launch', command)
        config = yaml.safe_load((Path(get_package_share_directory('slam')) / 'rviz/slam.rviz').read_text())
        config['Visualization Manager']['Views']['Current'].update(
            Scale=25.0 if args.mode == 'simulation' else 18.0,
            X=-0.4 if args.mode == 'simulation' else 8.0,
            Y=-0.5 if args.mode == 'simulation' else 0.0)
        config['Window Geometry'].update(Width=1400, Height=1000, X=0, Y=0)
        for display in config['Visualization Manager']['Displays']:
            if display['Class'] in ('rviz_default_plugins/LaserScan', 'rviz_default_plugins/PoseWithCovariance'):
                display['Enabled'] = False
        rviz_config = output / 'view.rviz'
        rviz_config.write_text(yaml.safe_dump(config, sort_keys=False))
        launch('rviz', ['/opt/ros/humble/lib/rviz2/rviz2', '-d', str(rviz_config),
                        '--ros-args', '-p', 'use_sim_time:=true'])
        last_report, stable_since = 0, None
        startup_reset = None
        armed = args.mode != 'simulation'
        finish = 'timeout'
        while time.monotonic() - started < args.timeout:
            pump(0.1)
            if stop_requested or (output / 'STOP').exists():
                finish = 'requested_stop'
                break
            if slam.poll() is not None:
                raise RuntimeError('SLAM launch exited: ' + str(slam.returncode))
            if (args.mode == 'simulation' and node.last_scan_wall is not None
                    and time.monotonic() - node.last_scan_wall > 30):
                raise RuntimeError('Sensor stream stopped for more than 30 wall seconds')
            if node.status.get('adapter_error'):
                raise RuntimeError(node.status['adapter_error'])
            elapsed = 0 if node.first_stamp is None else node.last_stamp - node.first_stamp
            if not armed:
                if startup_reset is None and node.status.get('processed_scans', 0) > 0 and node.status.get('queued_scans') == 0:
                    reset_client = node.create_client(Trigger, '/slam/reset')
                    if reset_client.service_is_ready():
                        result['startup'] = {'scans': len(node.scans), 'status': node.status}
                        startup_reset = reset_client.call_async(Trigger.Request())
                elif startup_reset is not None and startup_reset.done():
                    if not startup_reset.result().success:
                        raise RuntimeError('Stationary startup reset failed')
                    node.scans.clear()
                    node.first_stamp = None
                    node.stop_driving = False
                    armed = True
            if time.monotonic() - last_report > 20:
                print(json.dumps({'sim_s': round(elapsed, 1), 'wall_s': round(time.monotonic()-started),
                                  'scans': len(node.scans), 'status': node.status,
                                  'pose': node.paths.get('odom', [None])[-1:] if node.paths else [],
                                  'controller': node.follower.state if node.follower else None}), flush=True)
                (output / 'progress.json').write_text(json.dumps({'elapsed_s': elapsed, 'status': node.status,
                                                                 'paths': node.paths}, indent=2))
                last_report = time.monotonic()
            if args.mode == 'simulation' and elapsed >= args.max_sim_seconds:
                finish = 'simulation_duration'
                break
            if args.route and node.follower.done:
                finish = 'route_complete'
                break
            if args.mode == 'bag' and node.last_stamp is not None:
                at_end = node.last_stamp >= result['bag']['last_scan_stamp'] - 1e-5
                if at_end and node.status.get('queued_scans') == 0:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since > 4:
                        finish = 'bag_end'
                        break
                else:
                    stable_since = None
        node.stop_driving = True
        pump(2.0)
        # Freeze the estimator before collecting its final map/path snapshot.
        # Otherwise Gazebo can add a stationary scan while screenshots compress.
        pause = node.create_client(Trigger, '/slam/pause')
        if not pause.wait_for_service(timeout_sec=10):
            raise RuntimeError('pause service unavailable')
        paused = pause.call_async(Trigger.Request())
        deadline = time.monotonic()+30
        while not paused.done() and time.monotonic() < deadline:
            pump(.1)
        if not paused.done() or not paused.result().success:
            raise RuntimeError('Could not freeze final estimator snapshot')
        pump(.2)
        client = node.create_client(Trigger, '/slam/save_map')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('save_map service unavailable')
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 30
        while not future.done() and time.monotonic() < deadline:
            pump(0.1)
        response = future.result() if future.done() else None
        if response is None or not response.success:
            raise RuntimeError('Map export failed: ' + str(response))
        pump(3.0)
        app.processEvents()
        if not app.primaryScreen().grabWindow(0).save(str(output / 'rviz.png')):
            raise RuntimeError('RViz capture failed')
        result.update(success=finish in ('simulation_duration', 'bag_end', 'route_complete'), finish_reason=finish,
                      final_status=node.status, scan_messages_observed=len(node.scans),
                      elapsed_sensor_s=(node.last_stamp - node.first_stamp) if node.scans else 0,
                      wall_seconds=time.monotonic()-started, paths=node.paths,
                      status_history=node.status_history,
                      controller_transitions=node.follower.transitions if node.follower else [],
                      commands=node.commands)
    except BaseException as exc:
        result['error'] = type(exc).__name__ + ': ' + str(exc)
        raise
    finally:
        if node is not None:
            node.stop_driving = True
            with suppress(Exception):
                node.drive()
            result.setdefault('final_status', node.status)
            result.setdefault('paths', node.paths)
            if node.scans:
                size = max(len(s[-1]) for s in node.scans)
                ranges = np.full((len(node.scans), size), np.nan, dtype=np.float32)
                for i, row in enumerate(node.scans):
                    ranges[i, :len(row[-1])] = row[-1]
                np.savez_compressed(output / 'scans.npz',
                                    metadata=np.asarray([s[:-1] for s in node.scans]), ranges=ranges)
        # Only signal our own launch processes; ROS launch forwards SIGINT itself.
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
        result['exit_codes'] = [p.returncode for p in processes]
        (output / 'samples.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        for log in logs:
            log.close()
        if node is not None:
            with suppress(Exception):
                node.destroy_node()
        if rclpy.ok():
            with suppress(Exception):
                rclpy.shutdown()


if __name__ == '__main__':
    main()
