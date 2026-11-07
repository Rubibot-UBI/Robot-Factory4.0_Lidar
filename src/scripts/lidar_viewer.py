#!/usr/bin/env python3
"""Vista de cima em tempo real do /scan (referencial do laser: frente = cima, esquerda = esquerda).

    ros2 run my_bot lidar_viewer.py

Dependências: sudo apt install python3-matplotlib python3-tk
"""
import threading
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

VIEW = 2.0         # metade do lado da vista (m); a vista fica sempre centrada no laser (0, 0)
ACCUMULATE = 1     # nº de voltas sobrepostas (aumenta para ver a dispersão do ruído)
MIN_RANGE = 0.12   # ignora leituras junto ao mínimo do sensor

# Footprint do robô (m). O laser está 12.4 mm atrás do centro do robô.
ROBOT_LEN, ROBOT_WID, LASER_BEHIND_CENTER = 0.0963, 0.080, 0.0124


class Viewer(Node):
    def __init__(self):
        super().__init__('lidar_viewer')
        self.scans = deque(maxlen=ACCUMULATE)  # cada item: pontos Nx2 (horizontal = y, vertical = x)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        r = np.array(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r >= max(MIN_RANGE, msg.range_min)) & (r <= msg.range_max)
        r, a = r[ok], a[ok]
        self.scans.append(np.column_stack((r * np.sin(a), r * np.cos(a))))


def main():
    rclpy.init()
    node = Viewer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    ax.set_xlim(VIEW, -VIEW)  # horizontal = y do robô, invertido (esquerda à esquerda)
    ax.set_ylim(-VIEW, VIEW)  # vertical = x do robô (frente para cima)
    ax.set_xlabel('y (m)  ← esquerda')
    ax.set_ylabel('x (m)  ↑ frente')
    ax.grid(alpha=0.3)

    # Robô (retângulo) e posição do laser
    ax.add_patch(Rectangle((ROBOT_WID / 2, LASER_BEHIND_CENTER - ROBOT_LEN / 2), -ROBOT_WID, ROBOT_LEN,
                           fill=False, ec='black', lw=1.5))
    ax.plot([0], [0], 'k+', ms=8)

    points = ax.scatter([], [], s=14, c='tab:blue')

    def update(_):
        scans = list(node.scans)
        if scans:
            points.set_offsets(np.vstack(scans))
            ax.set_title(f'{len(scans[-1])} pontos válidos')
        return (points,)

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()