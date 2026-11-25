"""Nodo que republica la cámara a baja frecuencia, con pares color-profundidad.

Entradas (por defecto, los tópicos del RosMaster y del simulador):
`/cam_1/color/image_raw`, `/cam_1/color/camera_info`,
`/cam_1/depth/image_raw`, `/cam_1/depth/camera_info`.

Salidas, con el prefijo `output_prefix` (por defecto `/mission/cam`):
`<prefijo>/color/image_raw`, `<prefijo>/color/camera_info`,
`<prefijo>/depth/image_raw`, `<prefijo>/depth/camera_info`.

Cada `camera_info` se publica junto con su imagen y con el mismo tiempo, así
un bag grabado con estos tópicos alcanza para ubicar los conos offline.

El ancho de banda del wifi solo baja si este nodo corre en el robot: si corre
en la PC, la imagen completa igual viaja por wifi hasta él. No requiere
compilar: desde la carpeta `mission/` del repo,

    python3 -m mission.ros.camera_throttle_node --ros-args -p rate_hz:=3.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ..core.throttle import PairedThrottle


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class CameraThrottleNode(Node):
    def __init__(self):
        super().__init__('camera_throttle')
        defaults = {
            'rate_hz': 3.0,
            # Distancia máxima en tiempo entre un color y la profundidad que lo acompaña.
            'pair_tolerance': 0.05,
            'use_depth': True,
            'color_topic': '/cam_1/color/image_raw',
            'color_info_topic': '/cam_1/color/camera_info',
            'depth_topic': '/cam_1/depth/image_raw',
            'depth_info_topic': '/cam_1/depth/camera_info',
            'output_prefix': '/mission/cam',
            'stats_period': 5.0,
        }
        self.options = {name: self.declare_parameter(name, value).value
                        for name, value in defaults.items()}
        rate = float(self.options['rate_hz'])
        if rate <= 0:
            raise ValueError('rate_hz must be positive')
        self.throttle = PairedThrottle(1.0 / rate, float(self.options['pair_tolerance']))
        self.info = {'color': None, 'depth': None}

        # Entrada best-effort: se conecta tanto a publicadores best-effort como
        # reliable. Salida reliable: la acepta cualquier suscriptor (RViz, rosbag).
        out_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        prefix = self.options['output_prefix'].rstrip('/')
        self.image_pub = {'color': self.create_publisher(Image, f'{prefix}/color/image_raw', out_qos)}
        self.info_pub = {'color': self.create_publisher(CameraInfo, f'{prefix}/color/camera_info', out_qos)}
        self.create_subscription(Image, self.options['color_topic'], self.on_color, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.options['color_info_topic'],
                                 lambda m: self.on_info('color', m), qos_profile_sensor_data)
        if self.options['use_depth']:
            self.image_pub['depth'] = self.create_publisher(Image, f'{prefix}/depth/image_raw', out_qos)
            self.info_pub['depth'] = self.create_publisher(CameraInfo, f'{prefix}/depth/camera_info', out_qos)
            self.create_subscription(Image, self.options['depth_topic'], self.on_depth,
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, self.options['depth_info_topic'],
                                     lambda m: self.on_info('depth', m), qos_profile_sensor_data)
        self.create_timer(float(self.options['stats_period']), self.log_stats)
        self.get_logger().info(f'Republishing camera at {rate:.1f} Hz under {prefix}')

    def on_info(self, channel, message):
        self.info[channel] = message

    def on_color(self, message):
        self.publish(self.throttle.on_primary(seconds(message.header.stamp), message))

    def on_depth(self, message):
        self.publish(self.throttle.on_secondary(seconds(message.header.stamp), message))

    def publish(self, items):
        for kind, message in items:
            channel = 'color' if kind == 'primary' else 'depth'
            self.image_pub[channel].publish(message)
            info = self.info[channel]
            if info is not None:
                # La calibración casi nunca cambia; se republica con el tiempo de la imagen.
                info.header.stamp = message.header.stamp
                self.info_pub[channel].publish(info)

    def log_stats(self):
        s = self.throttle.stats
        missing = [c for c in self.info_pub if self.info[c] is None]
        self.get_logger().info(
            f'color {s.primary_received} in / {s.primary_passed} out, '
            f'depth {s.secondary_received} in / {s.paired} paired, {s.unpaired} unpaired'
            + (f'; no camera_info yet for {missing}' if missing else ''))


def main(args=None):
    rclpy.init(args=args)
    node = CameraThrottleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
