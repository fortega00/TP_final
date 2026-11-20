#!/usr/bin/env python3
"""Prueba en vivo de la localización en el simulador (Gazebo Classic).

Con el simulador y el nodo de localización ya lanzados:

1. lee la pose real del simulador (`/odom` en Classic);
2. publica un `/initialpose` corrido respecto de esa pose, como un click impreciso;
3. opcionalmente maneja al robot ida y vuelta en línea recta (`--drive`) o le
   manda un objetivo y espera a que llegue (`--goal`);
4. compara `/nav/pose` contra la pose real e imprime el error.

Solo sirve en simulación: en el robot real no hay pose real.
"""
import argparse
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def row(message_pose, stamp):
    p = message_pose
    return [stamp.sec + stamp.nanosec * 1e-9, p.position.x, p.position.y, yaw_of(p.orientation)]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--offset", type=float, nargs=3, default=[0.3, -0.2, 0.15], metavar=("DX", "DY", "DYAW"))
    p.add_argument("--drive", type=float, default=0.0, help="metros a recorrer ida y vuelta; 0 no maneja")
    p.add_argument("--speed", type=float, default=0.2)
    p.add_argument("--goal", type=float, nargs=3, action="append", metavar=("X", "Y", "YAW"),
                   help="objetivo en el mapa; se puede repetir")
    p.add_argument("--goal-timeout", type=float, default=180.0, help="segundos de simulación por objetivo")
    p.add_argument("--truth-topic", default="/odom")
    p.add_argument("--output", help="JSON con las dos trayectorias")
    args = p.parse_args()

    rclpy.init()
    node = Node("sim_live_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
    truth, estimate, status = [], [], []
    navigation = []
    node.create_subscription(Odometry, args.truth_topic,
                             lambda m: truth.append(row(m.pose.pose, m.header.stamp)), 50)
    node.create_subscription(PoseWithCovarianceStamped, "/nav/pose",
                             lambda m: estimate.append(row(m.pose.pose, m.header.stamp)), 50)
    node.create_subscription(String, "/nav/localization_status",
                             lambda m: status.append(json.loads(m.data)), 10)
    node.create_subscription(String, "/nav/state", lambda m: navigation.append(json.loads(m.data)), 10)
    goal_pub = node.create_publisher(PoseStamped, "/goal_pose", 10)
    initial = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    velocity = node.create_publisher(Twist, "/cmd_vel", 10)

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def spin(duration, command=None):
        end = now() + duration
        while rclpy.ok() and now() < end:
            if command is not None:
                velocity.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)

    while not truth or now() == 0:
        rclpy.spin_once(node, timeout_sec=0.2)
    spin(1.0)
    _, x, y, yaw = truth[-1]
    click = PoseWithCovarianceStamped()
    click.header.frame_id = "map"
    click.header.stamp = node.get_clock().now().to_msg()
    click.pose.pose.position.x, click.pose.pose.position.y = x + args.offset[0], y + args.offset[1]
    click.pose.pose.orientation.z = math.sin((yaw + args.offset[2]) / 2)
    click.pose.pose.orientation.w = math.cos((yaw + args.offset[2]) / 2)
    # Covarianza que manda 2D Pose Estimate de RViz: 0,5 m y 15 grados de desvío.
    click.pose.covariance[0] = click.pose.covariance[7] = 0.25
    click.pose.covariance[35] = (math.pi / 12) ** 2
    initial.publish(click)
    spin(3.0)
    if not estimate:
        raise SystemExit("No /nav/pose after the initial pose: is the localization node running?")
    first = len(estimate)

    if args.drive > 0:
        forward, turn = Twist(), Twist()
        forward.linear.x, turn.angular.z = args.speed, 0.5
        for _ in range(2):                       # ida, media vuelta, vuelta, media vuelta
            spin(args.drive / args.speed, forward)
            spin(1.0, Twist())
            spin(math.pi / 0.5, turn)
            spin(1.0, Twist())
        spin(2.0, Twist())

    arrivals = []
    for gx, gy, gyaw in args.goal or []:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = gx, gy
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        goal_pub.publish(goal)
        started, seen_moving = now(), False
        while rclpy.ok() and now() - started < args.goal_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            state = navigation[-1]["state"] if navigation else None
            seen_moving = seen_moving or state in ("PLANNING", "ALIGNING", "FOLLOWING")
            if seen_moving and state in ("IDLE", "WAIT_INITIAL_POSE", "WAIT_SCAN"):
                break
        spin(1.0)
        _, x, y, yaw = truth[-1]
        arrivals.append(dict(goal=[gx, gy, gyaw], seconds=round(now() - started, 1),
                             final_state=navigation[-1]["state"] if navigation else None,
                             last_events=navigation[-1]["events"][-2:] if navigation else None,
                             true_error_cm=round(100 * math.hypot(x - gx, y - gy), 1),
                             true_yaw_error_deg=round(math.degrees(abs(math.atan2(
                                 math.sin(yaw - gyaw), math.cos(yaw - gyaw)))), 1)))

    t = np.array(truth)
    e = np.array(estimate)
    inside = (e[:, 0] >= t[0, 0]) & (e[:, 0] <= t[-1, 0])
    e = e[inside]
    reference = np.column_stack([np.interp(e[:, 0], t[:, 0], t[:, i]) for i in (1, 2)])
    error = np.hypot(*(e[:, 1:3] - reference).T)
    after_click = error[:max(1, first)]
    result = dict(click_error_cm=round(100 * math.hypot(*args.offset[:2]), 1),
                  after_burst_cm=round(100 * float(after_click[-1]), 1),
                  rmse_cm=round(100 * float(np.sqrt(np.mean(error ** 2))), 1),
                  max_cm=round(100 * float(error.max()), 1),
                  final_cm=round(100 * float(error[-1]), 1),
                  poses=int(len(e)), arrivals=arrivals, status=status[-1] if status else None)
    print(json.dumps(result))
    if args.output:
        with open(args.output, "w") as handle:
            json.dump(dict(truth=t.tolist(), estimate=e.tolist(), result=result), handle)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
