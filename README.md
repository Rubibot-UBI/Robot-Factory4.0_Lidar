# My robot @ Factory 4.0

Robô diferencial (2 rodas motorizadas + 2 casters esféricos + LIDAR RPLidar) desenhado no Onshape.
ROS 2 Jazzy + Gazebo Harmonic.

## Requisitos

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard ros-jazzy-xacro
```

## Exportar o CAD do Onshape (só quando o CAD mudar)

```bash
cp .env_template .env     # preencher ONSHAPE_ACCESS_KEY e ONSHAPE_SECRET_KEY
uv sync                   # instala as dependências Python (onshape-to-robot)
uv run onshape-to-robot . # gera robot.urdf e as meshes a partir do config.json
```

Depois de exportar, copiar os `.stl` para `src/description/meshes/`.

> O `robot.urdf` gerado **não é usado diretamente**. A fonte de verdade é o xacro em
> `src/description/`. Se o CAD mudar (posições, peças novas), é preciso atualizar
> o `robot_core.xacro` à mão com os valores novos do `robot.urdf`.

## Compilar

O workspace é a pasta deste repositório. Na raiz (onde estão `src/`, `build/` e `install/`):

```bash
cd ~/dev_ws                       # ajustar para a pasta do workspace
colcon build --symlink-install
source install/setup.bash
```

Build limpo (se algo ficar estranho): `rm -rf build install log` antes do `colcon build`.
É preciso voltar a compilar quando se acrescentam ficheiros novos (meshes, launch, etc.).

> Cada terminal novo precisa de `source ~/dev_ws/install/setup.bash`.
> Para ser automático: `echo "source ~/dev_ws/install/setup.bash" >> ~/.bashrc`

## Ver o modelo no RViz

```bash
# terminal 1
source ~/dev_ws/install/setup.bash
ros2 launch my_bot rsp.launch.py

# terminal 2
source ~/dev_ws/install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# terminal 3
source ~/dev_ws/install/setup.bash
rviz2
```

No RViz: **Fixed Frame** = `base_link`, **Add → RobotModel**, **Description Topic** = `/robot_description`.

## Simular no Gazebo

```bash
# terminal 1
source ~/dev_ws/install/setup.bash
ros2 launch my_bot sim.launch.py

# terminal 2
source ~/dev_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Verificar: `ros2 topic echo /odom` (o `x` cresce a andar em frente) e `ros2 topic echo /scan --once`.

## Estrutura

| Pasta / ficheiro | Para quê |
|---|---|
| `src/description/robot_core.xacro` | links, joints, massas, materiais |
| `src/description/gazebo_control.xacro` | plugin diff_drive, joint_states, atrito e sensor LIDAR (Gazebo) |
| `tools/mass_budget.py` | recalcula massa, centro de massa e inércia do chassis a partir das meshes |
| `src/description/meshes/` | `.stl` exportados do Onshape |
| `src/launch/rsp.launch.py` | robot_state_publisher (RViz) |
| `src/launch/sim.launch.py` | Gazebo + spawn + ponte ROS↔Gazebo |
| `src/worlds/empty.world` | mundo vazio |

## Física do robô (o que está modelado e porquê)

Frames: `base_link` = ponto médio do eixo das rodas, à altura do eixo (x frente, y esquerda, z cima).
`chassis` = frame da placa base do CAD (frente do CAD = -Y, por isso `chassis_joint` roda 90° em Z).
Todas as poses das meshes são relativas à placa base. O export do Onshape vinha rodado
2.4° em Z e 1° em Y em relação ao frame do assembly, o que deixava os eixos das rodas tortos.

| Medido no CAD | Valor |
|---|---|
| Rodas | Ø 56 mm × 8 mm, centros a y = ±45.4 mm → `wheel_separation` = 0.0908 m |
| Esferas (casters) | Ø 12 mm, fundo à mesma cota que o fundo das rodas (3 apoios coplanares) |
| Dimensão do robô | placa 80 × 96 mm; ~110 mm de comprimento com o íman; 144 mm de altura (cabe nos 30×30×30 cm) |
| Eixo do LIDAR | 12.4 mm **atrás** do centro do robô (`lidar_x` em robot_core.xacro) |

Estimativas que **tens de confirmar com o robô real** (propriedades no topo de `robot_core.xacro`):
massas (`chassis_mass` 0.485 kg, rodas 12 g, esferas 7 g), limites do motor (`motor_max_torque`,
`motor_max_speed`) e a altura do plano de varrimento do LIDAR (`lidar_z`).
Depois de pesar, edita a tabela `PARTS` em `tools/mass_budget.py`, corre-o e copia os valores impressos.

### LIDAR
RPLIDAR A1M8: 0.15–12 m, 360 pontos, 10 Hz, ruído gaussiano σ = 5 mm, tópico `/scan`, frame `laser_frame`
(eixo X do laser = frente do robô). Parâmetros no topo de `gazebo_control.xacro`.
O mundo precisa do plugin `gz-sim-sensors-system` (já incluído em `empty.world`).

> **Atenção:** as paredes das máquinas e armazéns têm 130 mm de altura (figuras 4.2 e 4.3 das regras) e o plano do
> LIDAR está a ~130 mm do chão, mesmo no topo delas. Confirma a altura real do plano no datasheet/robô e,
> se necessário, baixa o LIDAR ou ajusta `lidar_z`.

## Por fazer
primeiro eu quero deixar tudo da parte "fisica" do robo bem, depois vou querer mudar o mapa porque eu vou fazer amcl, o mapa vai ser aquele que está nas regras da competição + uns pilares nos locais indicados, e so depois vou fazer eu o algoritmo em python para reagir aos valores dados do lidar, não esquecer de configurar este lidar de acordo com este link: