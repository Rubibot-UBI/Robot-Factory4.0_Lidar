#!/usr/bin/env python3
"""Gráficos em tempo real dos encoders (/joint_states) e do IMU (/imu).

    ros2 run my_bot sensors_viewer.py

Encoders: lê a posição das juntas `left_wheel_joint` e `right_wheel_joint` do /joint_states
(no simulador vem do Gazebo; no robô real basta o teu nó publicar um sensor_msgs/JointState igual)
e converte para contagens do encoder com ENCODER_CPR.
IMU: lê o /imu (sensor_msgs/Imu): giroscópio, acelerómetro e inclinação/guinada estimadas.

Dependências: sudo apt install python3-matplotlib python3-tk
"""
import math
import threading
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState

WINDOW_S = 10.0          # segundos de histórico mostrados
# Contagens por volta da RODA. ESTIMATIVA para o MOT02049 (50:1): 7 pulsos x 2 flancos x 2 canais x 50 = 1400.
# A página da loja é contraditória: mede (roda uma volta à mão) e corrige.
ENCODER_CPR = 1400
WHEEL_RADIUS = 0.028     # m (robot_core.xacro)
WHEEL_SEPARATION = 0.0908  # m (robot_core.xacro)
RATED_RPM = 180          # rotação nominal do motor a 6 V (linha de referência no gráfico)
LEFT_JOINT, RIGHT_JOINT = 'left_wheel_joint', 'right_wheel_joint'
RAD2DEG = 180.0 / math.pi
G = 9.81


class Viewer(Node):
    def __init__(self):
        super().__init__('sensors_viewer')
        self.lock = threading.Lock()
        self.t0 = None
        # encoders
        self.enc_t = deque()
        self.enc_ticks = {'L': deque(), 'R': deque()}
        self.enc_rpm = {'L': deque(), 'R': deque()}
        self._last_pos = None  # (t, posL, posR) para calcular velocidade se o JointState não a trouxer
        self.ticks_zero = None
        self.last_enc_msg = None
        # imu
        self.imu_t = deque()
        self.gyro = [deque(), deque(), deque()]   # deg/s
        self.acc = [deque(), deque(), deque()]    # m/s^2
        self.roll = deque()
        self.pitch = deque()
        self.yaw = deque()                        # integrado do giroscópio z: deriva (BMI160 não tem magnetómetro)
        self._yaw = 0.0
        self._last_imu_t = None
        self.last_imu_msg = None
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)

    def _time(self, stamp):
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    @staticmethod
    def _trim(t_deque, *series):
        while t_deque and t_deque[-1] - t_deque[0] > WINDOW_S:
            t_deque.popleft()
            for s in series:
                s.popleft()

    def on_joints(self, msg):
        if LEFT_JOINT not in msg.name or RIGHT_JOINT not in msg.name:
            return
        iL, iR = msg.name.index(LEFT_JOINT), msg.name.index(RIGHT_JOINT)
        with self.lock:
            t = self._time(msg.header.stamp)
            pos = (msg.position[iL], msg.position[iR])
            if self.ticks_zero is None:
                self.ticks_zero = pos  # contagens relativas ao arranque do viewer
            if len(msg.velocity) > max(iL, iR):
                vel = (msg.velocity[iL], msg.velocity[iR])
            elif self._last_pos and t > self._last_pos[0]:
                dt = t - self._last_pos[0]
                vel = ((pos[0] - self._last_pos[1]) / dt, (pos[1] - self._last_pos[2]) / dt)
            else:
                vel = (0.0, 0.0)
            self._last_pos = (t, *pos)
            self.enc_t.append(t)
            for k, i in (('L', 0), ('R', 1)):
                self.enc_ticks[k].append((pos[i] - self.ticks_zero[i]) / (2 * math.pi) * ENCODER_CPR)
                self.enc_rpm[k].append(vel[i] * 60 / (2 * math.pi))
            self._trim(self.enc_t, *self.enc_ticks.values(), *self.enc_rpm.values())
            self.last_enc_msg = (pos[0] - self.ticks_zero[0], pos[1] - self.ticks_zero[1], vel)

    def on_imu(self, msg):
        with self.lock:
            t = self._time(msg.header.stamp)
            w, a = msg.angular_velocity, msg.linear_acceleration
            if self._last_imu_t is not None and t > self._last_imu_t:
                self._yaw += w.z * (t - self._last_imu_t)
            self._last_imu_t = t
            self.imu_t.append(t)
            for s, v in zip(self.gyro, (w.x, w.y, w.z)):
                s.append(v * RAD2DEG)
            for s, v in zip(self.acc, (a.x, a.y, a.z)):
                s.append(v)
            # inclinação a partir da gravidade (só vale com o robô parado/a andar suave)
            self.roll.append(math.atan2(a.y, a.z) * RAD2DEG)
            self.pitch.append(math.atan2(-a.x, math.hypot(a.y, a.z)) * RAD2DEG)
            self.yaw.append(self._yaw * RAD2DEG)
            self._trim(self.imu_t, *self.gyro, *self.acc, self.roll, self.pitch, self.yaw)
            self.last_imu_msg = (w, a)


def main():
    rclpy.init()
    node = Viewer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, axs = plt.subplots(3, 2, figsize=(13, 9), sharex=False)
    fig.subplots_adjust(hspace=0.45, wspace=0.22, top=0.93, bottom=0.07)
    (ax_tk, ax_gy), (ax_rpm, ax_ac), (ax_txt, ax_att) = axs

    def setup(ax, title, ylabel):
        ax.set_title(title, fontsize=10)
        ax.set_ylabel(ylabel)
        ax.set_xlabel('t (s)')
        ax.grid(alpha=0.3)

    setup(ax_tk, f'Encoders: contagens (CPR = {ENCODER_CPR}/volta)', 'contagens')
    setup(ax_rpm, 'Rotação das rodas', 'rpm')
    setup(ax_gy, 'IMU: giroscópio', '°/s')
    setup(ax_ac, 'IMU: acelerómetro', 'm/s²')
    setup(ax_att, 'IMU: roll / pitch (acelerómetro) e yaw (integrado, deriva)', 'graus')
    ax_txt.axis('off')

    colors = {'L': 'tab:blue', 'R': 'tab:red'}
    l_tk = {k: ax_tk.plot([], [], c=colors[k], label='esquerda' if k == 'L' else 'direita')[0] for k in 'LR'}
    l_rpm = {k: ax_rpm.plot([], [], c=colors[k], label='esquerda' if k == 'L' else 'direita')[0] for k in 'LR'}
    ax_rpm.axhline(RATED_RPM, c='gray', ls='--', lw=0.7)
    ax_rpm.axhline(-RATED_RPM, c='gray', ls='--', lw=0.7)
    l_gy = [ax_gy.plot([], [], label=n)[0] for n in 'xyz']
    l_ac = [ax_ac.plot([], [], label=n)[0] for n in 'xyz']
    l_att = [ax_att.plot([], [], label=n)[0] for n in ('roll', 'pitch', 'yaw')]
    for ax in (ax_tk, ax_rpm, ax_gy, ax_ac, ax_att):
        ax.legend(loc='upper left', fontsize=8)
    txt = ax_txt.text(0, 1, '', va='top', family='monospace', fontsize=10)

    def update(_):
        with node.lock:
            et = np.array(node.enc_t)
            for k in 'LR':
                l_tk[k].set_data(et, np.array(node.enc_ticks[k]))
                l_rpm[k].set_data(et, np.array(node.enc_rpm[k]))
            it = np.array(node.imu_t)
            for line, s in zip(l_gy, node.gyro):
                line.set_data(it, np.array(s))
            for line, s in zip(l_ac, node.acc):
                line.set_data(it, np.array(s))
            for line, s in zip(l_att, (node.roll, node.pitch, node.yaw)):
                line.set_data(it, np.array(s))
            enc, imu = node.last_enc_msg, node.last_imu_msg

            lines = []
            if enc:
                pL, pR, (vL, vR) = enc
                v = WHEEL_RADIUS * (vL + vR) / 2
                w = WHEEL_RADIUS * (vR - vL) / WHEEL_SEPARATION
                lines += [f'Encoder  L: {pL / (2 * math.pi) * ENCODER_CPR:9.0f} cont  {vL * 60 / (2 * math.pi):6.1f} rpm',
                          f'         R: {pR / (2 * math.pi) * ENCODER_CPR:9.0f} cont  {vR * 60 / (2 * math.pi):6.1f} rpm',
                          f'Odometria das rodas: v = {v:+.3f} m/s   w = {w * RAD2DEG:+.1f} °/s', '']
            else:
                lines += ['Encoders: sem dados em /joint_states', '']
            if imu:
                w, a = imu
                mag = math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)
                lines += [f'Gyro  x={w.x * RAD2DEG:+7.2f} y={w.y * RAD2DEG:+7.2f} z={w.z * RAD2DEG:+7.2f} °/s',
                          f'Accel x={a.x:+7.2f} y={a.y:+7.2f} z={a.z:+7.2f} m/s²  |a|={mag:.2f} (g={G})']
            else:
                lines += ['IMU: sem dados em /imu']
            txt.set_text('\n'.join(lines))

        for ax, ts in ((ax_tk, et), (ax_rpm, et), (ax_gy, it), (ax_ac, it), (ax_att, it)):
            if len(ts):
                ax.set_xlim(max(0.0, ts[-1] - WINDOW_S), max(WINDOW_S, ts[-1]))
            ax.relim()
            ax.autoscale_view(scalex=False)
        return []

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
