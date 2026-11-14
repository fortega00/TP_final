"""Diagnostic trajectories only: never feed ground truth back into SLAM."""
from collections import deque

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformException

from ..core.geometry import Pose2D
from ..core.odometry import OdometryBuffer


class ReferencePaths:
    def __init__(self, node, persistent, pose_message, seconds, yaw):
        self.node = node
        self.pose_message, self.seconds, self.yaw = pose_message, seconds, yaw
        self.limit = node.options['max_path_poses']
        self.frame = node.options['map_frame']
        cfg = node.config.filter
        self.initial = Pose2D(cfg.initial_x, cfg.initial_y, cfg.initial_yaw)
        self.raw_pub = node.create_publisher(Path, '/slam/odom_path', persistent)
        self.truth_pub = node.create_publisher(Path, '/slam/ground_truth_path', persistent)
        self.enabled = bool(node.options['ground_truth_topic'])
        if self.enabled:
            kind = PoseStamped if node.options['mode'] == 'toy' else Odometry
            qos = QoSProfile(depth=max(100, node.options['scan_queue_size']),
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(kind, node.options['ground_truth_topic'],
                                     self.on_truth, qos)
        self.reset()

    def reset(self):
        self.raw = Path()
        self.truth = Path()
        self.raw.header.frame_id = self.truth.header.frame_id = self.frame
        self.buffer = OdometryBuffer(self.node.options['odom_history_seconds'])
        self.anchor_stamp = self.raw_alignment = self.truth_alignment = None
        self.source_frames = None
        self.pending = deque(maxlen=self.limit)
        self.raw_pub.publish(self.raw)
        self.truth_pub.publish(self.truth)

    def append(self, path, publisher, stamp, pose):
        point = PoseStamped()
        point.header.frame_id, point.header.stamp = self.frame, stamp
        point.pose = self.pose_message(pose)
        path.header.stamp = stamp
        path.poses.append(point)
        path.poses = path.poses[-self.limit:]
        publisher.publish(path)

    def observe(self, stamp, odom):
        if self.raw_alignment is None:
            self.anchor_stamp = self.seconds(stamp)
            self.raw_alignment = self.initial.compose(odom.inverse())
        # Freeze alignment at session start, independent of map -> odom TF.
        self.append(self.raw, self.raw_pub, stamp, self.raw_alignment.compose(odom))
        if self.enabled:
            self.pending.append(stamp)
            self.flush_truth()

    def on_truth(self, message):
        try:
            frames = (message.header.frame_id, getattr(message, 'child_frame_id', ''))
            if self.source_frames is not None and frames != self.source_frames:
                raise ValueError('Ground truth frames changed; reset the session')
            body = message.pose if isinstance(message, PoseStamped) else message.pose.pose
            pose = Pose2D(body.position.x, body.position.y, self.yaw(body.orientation))
            if not frames[0]:
                raise ValueError('Ground truth header.frame_id is empty')
            if isinstance(message, Odometry):
                if not message.child_frame_id:
                    raise ValueError('Ground truth child_frame_id is empty')
                pose = pose.compose(self.node.lookup(message.child_frame_id,
                                                     self.node.options['base_frame'],
                                                     message.header.stamp))
            stamp = self.seconds(message.header.stamp)
            if self.buffer.samples and stamp < self.buffer.samples[-1][0]:
                raise ValueError('Ground truth clock moved backwards; reset the session')
            if self.buffer.append(stamp, pose):
                self.source_frames = frames
                self.flush_truth()
        except (ValueError, TransformException) as exc:
            self.node.get_logger().warning(str(exc), throttle_duration_sec=5.0)

    def flush_truth(self):
        if self.anchor_stamp is None:
            return
        if self.truth_alignment is None:
            anchor = self.buffer.at(self.anchor_stamp)
            if anchor is None:
                self.node.get_logger().warning(
                    'Waiting for ground truth bracketing the first integrated scan; '
                    'start the reference source before mapping (or reset while stationary).',
                    throttle_duration_sec=10.0)
                return
            self.truth_alignment = self.initial.compose(anchor.inverse())
        while self.pending:
            stamp = self.pending[0]
            pose = self.buffer.at(self.seconds(stamp))
            if pose is None:
                if self.buffer.samples and self.seconds(stamp) < self.buffer.samples[0][0]:
                    self.pending.popleft()
                    continue
                break
            self.pending.popleft()
            self.append(self.truth, self.truth_pub, stamp, self.truth_alignment.compose(pose))
