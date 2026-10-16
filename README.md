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

Verificar: `ros2 topic echo /odom` (o `x` cresce a andar em frente).

## Estrutura

| Pasta / ficheiro | Para quê |
|---|---|
| `src/description/robot_core.xacro` | links, joints, massas, materiais |
| `src/description/gazebo_control.xacro` | plugin diff_drive e atrito (Gazebo) |
| `src/description/meshes/` | `.stl` exportados do Onshape |
| `src/launch/rsp.launch.py` | robot_state_publisher (RViz) |
| `src/launch/sim.launch.py` | Gazebo + spawn + ponte ROS↔Gazebo |
| `src/worlds/empty.world` | mundo vazio |

## Por fazer
primeiro eu quero deixar tudo da parte "fisica" do robo bem, depois vou querer mudar o mapa porque eu vou fazer amcl, o mapa vai ser aquele que está nas regras da competição + uns pilares nos locais indicados, e so depois vou fazer eu o algoritmo em python para reagir aos valores dados do lidar, não esquecer de configurar este lidar de acordo com este link: