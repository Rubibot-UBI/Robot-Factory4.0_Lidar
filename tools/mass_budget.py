#!/usr/bin/env python3
"""Calcula massa total, centro de massa e tensor de inercia do link `chassis`
a partir das meshes do CAD + uma tabela de massas.

Uso:  python3 tools/mass_budget.py
Depois copiar os valores impressos para `chassis_mass`, `chassis_com` e
`chassis_inertia_*` em src/description/robot_core.xacro.

>>> PESA O ROBO REAL e atualiza a tabela PARTS (massas em kg). <<<
Todas as poses estao no frame da placa base ("plate frame": origem no centro da
placa, z=0 na face de cima da zona de montagem, frente do robo = -Y).
"""
import os
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R

MESHES = os.path.join(os.path.dirname(__file__), '..', 'src', 'description', 'meshes')


def T(xyz, rpy):
    M = np.eye(4)
    M[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
    M[:3, 3] = xyz
    return M


# nome, mesh, pose no plate frame, massa [kg], usar convex hull? (meshes nao estanques)
PARTS = [
    ('base_plate', 'base.stl',   T([0, 0, 0], [0, 0, 0]),                     0.060, False),  # PLA ~35% enchimento (estimativa)
    ('body',       'resto.stl',  T([0, 0, 0.02], [0, 0, 0]),                  0.200, False),  # bloco = bateria + PCBs + caixa (PLACEHOLDER)
    ('motor_L',    'n20_mini_micro_metall_gear_motor.stl', T([0.0304, 0, 0.007], [0, 0, 0]),          0.011, True),   # N20 c/ caixa ~10-12 g
    ('motor_R',    'n20_mini_micro_metall_gear_motor.stl', T([-0.0304, 0, 0.007], [0, 0, -np.pi]),    0.011, True),
    ('magnet',     'eletroiman.stl', T([0, -0.04615, 0.004], [np.pi / 2, 0, 0]), 0.030, False),
    ('lidar',      'rp_lidar.stl',   T([0.13277, -0.07837, -0.64969], [np.pi / 2, 0, -np.pi / 2]), 0.170, True),  # RPLIDAR A1: 170 g (datasheet)
    ('caster_body_F', 'casterballwheelbody__cut_extrude6.stl', T([0, -0.03815, -0.0175], [0, 0, 0]),   0.0015, False),
    ('caster_body_B', 'casterballwheelbody__cut_extrude6.stl', T([0, 0.03613, -0.0174], [0, 0, 0]),    0.0015, False),
]

tot = 0.0
com = np.zeros(3)
items = []
for name, mesh, M, mass, hull in PARTS:
    m = trimesh.load(os.path.join(MESHES, mesh))
    if hull:
        m = m.convex_hull
    m = m.copy()
    m.apply_transform(M)
    c = m.center_mass
    # inercia p/ densidade 1 -> escalar para a massa real (I = I1 * mass/volume)
    I = m.moment_inertia * (mass / m.volume)
    items.append((name, mass, c, I))
    tot += mass
    com += mass * c
com /= tot

Itot = np.zeros((3, 3))
for name, mass, c, I in items:
    d = c - com
    Itot += I + mass * ((d @ d) * np.eye(3) - np.outer(d, d))

print(f'massa do chassis (sem rodas/esferas): {tot:.4f} kg')
print(f'centro de massa no plate frame [m]:   {com[0]:.5f} {com[1]:.5f} {com[2]:.5f}')
print('tensor de inercia no CM (plate frame) [kg m^2]:')
print(f'  ixx={Itot[0,0]:.4e} ixy={Itot[0,1]:.4e} ixz={Itot[0,2]:.4e}')
print(f'  iyy={Itot[1,1]:.4e} iyz={Itot[1,2]:.4e} izz={Itot[2,2]:.4e}')
