#!/usr/bin/env python3
"""Recebe o /scan do LIDAR e o /odom, e deixa tudo pronto para o AMCL.

Correr (com o sim.launch.py já a andar):
    ros2 run my_bot lidar_listener.py

O ponto de partida do AMCL é a função `process_scan`.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# O README diz para ignorar leituras junto ao mínimo do sensor (0.1 m): são ruidosas.
MIN_RANGE = 0.12


class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        # O /scan vem como "sensor data" (best-effort); /odom usa a QoS normal.
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.odom = (0.0, 0.0, 0.0)  # x, y, yaw (odom -> base_link), útil para o passo de predição do AMCL

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw)

    def scan_callback(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Fica só com leituras válidas (o simulador devolve inf quando não vê nada).
        valid = np.isfinite(ranges) & (ranges >= max(MIN_RANGE, msg.range_min)) & (ranges <= msg.range_max)
        self.process_scan(ranges[valid], angles[valid])

    def process_scan(self, ranges, angles):
        """Aqui entra o AMCL.

        ranges: distâncias válidas em metros (numpy)
        angles: ângulo de cada leitura em radianos, 0 = frente do robô, positivo = esquerda (numpy)
        self.odom: última pose da odometria (x, y, yaw)

        Nota: o laser está 12.4 mm atrás do centro do robô (lidar_x em robot_core.xacro).
        """
        if len(ranges) == 0:
            return
        closest = int(np.argmin(ranges))
        x, y, yaw = self.odom
        self.get_logger().info(
            f'{len(ranges)} pontos | mais perto: {ranges[closest]:.2f} m a {math.degrees(angles[closest]):.0f}° '
            f'| odom: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = LidarListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
