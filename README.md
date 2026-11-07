# My robot @ Factory 4.0

Robô diferencial (2 rodas + 2 casters + LIDAR LD14P/D200) para o RobotAtFactory 4.0. ROS 2 Jazzy + Gazebo Harmonic.

## Instalar e compilar

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard ros-jazzy-xacro python3-matplotlib python3-tk

cd ~/dev_ws                          # a raiz deste repositório (onde estão src/, build/, install/)
colcon build --symlink-install
source install/setup.bash            # em cada terminal novo
```

Recompilar sempre que se acrescentam ficheiros (meshes, launch, scripts). Build limpo: `rm -rf build install log`.
Scripts novos: acrescentar ao `install(PROGRAMS ...)` do `src/CMakeLists.txt`.

## Comandos

```bash
# --- terminal 1: simulação (pista + objetos aleatórios fora da pista) ---
ros2 launch my_bot sim.launch.py                                # seed aleatória, impressa no terminal
ros2 launch my_bot sim.launch.py seed:=42                       # mapa reproduzível
ros2 launch my_bot sim.launch.py seed:=42 object_counts:=pessoa=6,cone=0
ros2 launch my_bot sim.launch.py extra_objects:=false           # só o mundo base das regras

# --- terminal 2: conduzir ---
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# --- terminal 3: ver dados (um de cada vez) ---
ros2 run my_bot lidar_viewer.py      # vista de cima do /scan
ros2 run my_bot sensors_viewer.py    # encoders e IMU
ros2 run my_bot lidar_listener.py    # nó Python do AMCL (começa em process_scan)

# --- só ver o modelo no RViz (sem Gazebo) ---
ros2 launch my_bot rsp.launch.py
ros2 run joint_state_publisher_gui joint_state_publisher_gui
rviz2                                # Fixed Frame = base_link; Add → RobotModel (/robot_description)
```

Verificar: `ros2 topic echo /odom` (x cresce a andar em frente) e `ros2 topic echo /scan --once`.

## Mapa aleatório (objetos fora da pista)

O `sim.launch.py` gera um mundo novo a cada arranque: `factory.world` (pista, warehouses, máquinas e 6 pilares das regras,
sem alterações) + objetos aleatórios fora do chão da pista, como obstáculos que o mapa do AMCL não conhece.
Mesma seed = mesmo mapa. O `.world` e um `.json` com a verdade-terreno (x, y, yaw de cada objeto) ficam em
`/tmp/my_bot_world_<seed>.{world,json}`.

| `object_counts` | Altura | Quantidade |
|---|---|---|
| `pilar` | 0,21 m (como os das regras) | 2–5 |
| `pessoa` | 1,70 m, **encostadas à pista** (5–55 cm) | 4–8 |
| `cadeira` | 0,80 m | 1–4 |
| `mesa` | 0,75 m (o LIDAR só vê as pernas) | 1–2 |
| `quadrado` | 0,30 m | 1–4 |
| `cone` | 0,50 m | 2–5 |
| `balde` | 0,55 m | 1–3 |

Os outros ficam numa faixa de 2 m à volta da pista, a ≥ 0,20 m do chão, sem se tocarem. Tamanhos, quantidades e distâncias
estão no `CATALOG` de `src/worlds/world_generator.py` (as meshes vêm em unidades e origens diferentes; o catálogo normaliza escala,
pivô e frente). Fora do ROS:

```bash
python3 src/worlds/world_generator.py --seed 42 --preview mapa.png        # desenho 2D (matplotlib)
python3 src/worlds/world_generator.py --check-meshes src/description/meshes   # confere o CATALOG com os .stl (trimesh)
```

## Estrutura

| Ficheiro | Para quê |
|---|---|
| `src/description/robot_core.xacro` | links, joints, massas, materiais (fonte de verdade do robô) |
| `src/description/gazebo_control.xacro` | diff_drive, joint_states, atrito e sensor LIDAR (Gazebo) |
| `src/description/meshes/` | `.stl` do robô e dos objetos do mundo |
| `src/worlds/factory.world`, `world_generator.py` | mundo base e gerador dos objetos aleatórios |
| `src/launch/` | `sim.launch.py` (Gazebo + spawn + ponte ROS↔Gazebo), `rsp.launch.py` (RViz) |
| `src/scripts/` | `lidar_viewer.py`, `sensors_viewer.py`, `lidar_listener.py` |
| `tools/mass_budget.py` | recalcula massa, centro de massa e inércia do chassis a partir das meshes |

## Notas

- **Frames:** `base_link` = ponto médio do eixo das rodas (x frente, y esquerda). O LIDAR está 12,4 mm atrás do centro. Frame do laser: `laser_frame`, tópico `/scan`.
- **LIDAR D200:** 0,1–8 m, 6 Hz. No AMCL usar `laser_min_range` ≈ 0,12; abaixo de 0,1 m não mede nada.
- **Altura do plano do LIDAR:** ~135 mm do chão, entre as paredes das máquinas/armazéns (130 mm) e os pilares (210 mm). A altura do plano dentro do sensor (`lidar_plane_h` = 0,027 m) é uma estimativa: medir no sensor real; se ficar abaixo de ~140 mm do chão, subir o LIDAR.
- **A confirmar no robô real:** massas (`robot_core.xacro`, corrigir com `tools/mass_budget.py`), limites do motor (`motor_max_torque`, `motor_max_speed`) e `ENCODER_CPR` no topo do `sensors_viewer.py` (1400 é estimativa).
- **Exportar o CAD do Onshape** (só quando o CAD mudar): `cp .env_template .env` (chaves), `uv sync`, `uv run onshape-to-robot .`; copiar os `.stl` para `src/description/meshes/` e atualizar o xacro à mão (o `robot.urdf` exportado não é usado diretamente).

## Por fazer

- AMCL em Python (em `process_scan` do `lidar_listener.py`).
- Configurar o LIDAR de acordo com o link (em falta).