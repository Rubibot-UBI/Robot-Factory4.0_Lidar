#!/usr/bin/env python3
"""Guarda uma volta do LIDAR em CSV ao carregar em 's' (acrescenta se o ficheiro já existir).

Correr:  ros2 run my_bot guardar_scan.py
         ros2 run my_bot guardar_scan.py --ros-args -p ficheiro:=scans.csv
Teclas:  s = guardar a última volta | q = sair
"""
import csv
import os
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

CABECALHO = ['scan_id', 'stamp', 'angle', 'range', 'intensity']


class GuardarScan(Node):
    def __init__(self):
        super().__init__('guardar_scan')
        self.declare_parameter('ficheiro', 'scans.csv')
        self.ficheiro = self.get_parameter('ficheiro').value
        self.ultimo = None  # última volta recebida
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.proximo_id = self._proximo_id()
        self.get_logger().info(f"A escrever em '{self.ficheiro}' (próximo scan_id = {self.proximo_id}). "
                               "Carrega em 's' para guardar, 'q' para sair.")

    def on_scan(self, msg):
        self.ultimo = msg

    def _proximo_id(self):
        """Se o CSV já existe, continua a numeração a partir do último scan_id."""
        if not os.path.isfile(self.ficheiro) or os.path.getsize(self.ficheiro) == 0:
            return 0
        ultimo = -1
        with open(self.ficheiro, newline='') as f:
            for linha in csv.DictReader(f):
                ultimo = int(linha['scan_id'])
        return ultimo + 1

    def guardar(self):
        msg = self.ultimo
        if msg is None:
            self.get_logger().warn('Ainda não chegou nenhuma volta do /scan.')
            return
        novo = not os.path.isfile(self.ficheiro) or os.path.getsize(self.ficheiro) == 0
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with open(self.ficheiro, 'a', newline='') as f:   # 'a' = acrescenta
            w = csv.writer(f)
            if novo:
                w.writerow(CABECALHO)
            for i, r in enumerate(msg.ranges):
                inten = msg.intensities[i] if i < len(msg.intensities) else ''
                w.writerow([self.proximo_id, f'{stamp:.6f}',
                            f'{msg.angle_min + i * msg.angle_increment:.6f}', r, inten])
        self.get_logger().info(f'Volta guardada: scan_id={self.proximo_id} ({len(msg.ranges)} pontos)')
        self.proximo_id += 1


def ler_teclas(node, parar):
    """Lê teclas sem precisar de Enter (modo cbreak) e restaura o terminal no fim."""
    fd = sys.stdin.fileno()
    antigo = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not parar.is_set():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                tecla = sys.stdin.read(1).lower()
                if tecla == 's':
                    node.guardar()
                elif tecla == 'q':
                    parar.set()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, antigo)


def main():
    rclpy.init()
    node = GuardarScan()
    parar = threading.Event()
    t = threading.Thread(target=ler_teclas, args=(node, parar), daemon=True)
    t.start()
    try:
        while rclpy.ok() and not parar.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    parar.set()
    t.join()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()