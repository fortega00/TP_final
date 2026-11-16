"""Reproduce el avance sin scans nuevos invocando los callbacks reales de B.

No usa hardware, DDS ni tiempo de pared. Los publicadores se reemplazan por
listas y el reloj por un valor fijo. Ejecutar con los paquetes ROS cargados.
"""
import json
from types import SimpleNamespace

from nav_msgs.msg import Odometry

from slam.core.geometry import Pose2D
from slam.core.odometry import OdometryBuffer
from nav.core.config import NavConfig
from nav.core.follower import PurePursuit
from nav.core.navigator import Navigator, State
from nav.ros.localization_node import LocalizationNode
from nav.ros.navigator_node import NavigatorNode


config = NavConfig()
published_poses = []
# Representa una localización previamente convergida en t=1.
localizer = SimpleNamespace(
    odom_child='base_footprint', odometry=OdometryBuffer(30.0),
    correction=Pose2D(), state='LOCALIZED',
    publish_pose=lambda pose, stamp: published_poses.append((pose, stamp.sec)))
for stamp in (1, 100):
    message = Odometry()
    message.child_frame_id = 'base_footprint'
    message.header.stamp.sec = stamp
    message.pose.pose.orientation.w = 1.0
    LocalizationNode.on_odom(localizer, message)

core = Navigator(None, config)
core.state = State.FOLLOWING
core.goal = Pose2D(2.0, 0.0, 0.0)
core.follower = PurePursuit([(0.0, 0.0), (2.0, 0.0)], config.follower)
commands = []
node = SimpleNamespace(
    now=lambda: 100.0, pose=published_poses[-1][0],
    pose_stamp=float(published_poses[-1][1]),
    localization=localizer.state, blocked=False, core=core, config=config,
    was_active=True, shown_events=0, shown_path=core.path,
    velocity_pub=SimpleNamespace(publish=commands.append),
    state_pub=SimpleNamespace(publish=lambda message: None),
    get_logger=lambda: SimpleNamespace(info=lambda text: None))
NavigatorNode.step(node)
result = dict(
    seconds_without_scan=99, localization=localizer.state,
    published_pose_stamp=node.pose_stamp, navigator_state=core.state.value,
    linear_x=commands[-1].linear.x, angular_z=commands[-1].angular.z,
    kind='callback reproduction with fake publishers; not a physical test')
assert result['linear_x'] > 0, result
print(json.dumps(result, indent=2))
