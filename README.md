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

> O `robot.urdf` gerado (ignorado pelo git) **não é usado diretamente**. A fonte de verdade é o xacro em
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

## Receber o LIDAR em Python

```bash
# terminal 1: ros2 launch my_bot sim.launch.py   |   terminal 2: teleop   |   terminal 3:
ros2 run my_bot lidar_listener.py
```

O AMCL começa em `process_scan()` dentro de `src/scripts/lidar_listener.py`. Ao acrescentar scripts novos,
adicionar a linha correspondente `install(PROGRAMS ...)` no `src/CMakeLists.txt`.

## Estrutura

| Pasta / ficheiro | Para quê |
|---|---|
| `src/description/robot_core.xacro` | links, joints, massas, materiais |
| `src/description/gazebo_control.xacro` | plugin diff_drive, joint_states, atrito e sensor LIDAR (Gazebo) |
| `tools/mass_budget.py` | recalcula massa, centro de massa e inércia do chassis a partir das meshes |
| `src/description/meshes/` | `.stl` exportados do Onshape |
| `src/launch/rsp.launch.py` | robot_state_publisher (RViz) |
| `src/launch/sim.launch.py` | Gazebo + spawn + ponte ROS↔Gazebo |
| `src/worlds/factory.world` | mundo da competição: chão com a imagem da pista, 2 warehouses, 2 máquinas e 6 pilares (usado pelo `sim.launch.py`) |
| `src/worlds/textures/floor.png` | textura do chão (1700×1200 mm) |
| `src/scripts/lidar_listener.py` | nó Python que recebe `/scan` e `/odom`: ponto de partida para o AMCL |

## Física do robô (o que está modelado e porquê)

Frames: `base_link` = ponto médio do eixo das rodas, à altura do eixo (x frente, y esquerda, z cima).
`chassis` = frame da placa base do CAD (frente do CAD = -Y, por isso `chassis_joint` roda 90° em Z).
Todas as poses das meshes são relativas à placa base. O export do Onshape vinha rodado
2.4° em Z e 1° em Y em relação ao frame do assembly, o que deixava os eixos das rodas tortos.

| Medido no CAD | Valor |
|---|---|
| Rodas | Ø 56 mm × 8 mm, centros a y = ±45.4 mm → `wheel_separation` = 0.0908 m |
| Esferas (casters) | Ø 12 mm, fundo à mesma cota que o fundo das rodas (3 apoios coplanares) |
| Dimensão do robô | placa 80 × 96 mm; ~110 mm de comprimento com o íman; ~147 mm de altura com o D200 (cabe nos 30×30×30 cm) |
| Eixo do LIDAR | 12.4 mm **atrás** do centro do robô (`lidar_x` em robot_core.xacro) |

Estimativas que **tens de confirmar com o robô real** (propriedades no topo de `robot_core.xacro`):
massas (`chassis_mass` 0.485 kg, rodas 12 g, esferas 7 g), limites do motor (`motor_max_torque`,
`motor_max_speed`) e a altura do plano de varrimento do LIDAR (`lidar_z`).
Depois de pesar, edita a tabela `PARTS` em `tools/mass_budget.py`, corre-o e copia os valores impressos.

### LIDAR
LDROBOT LD14P (kit Waveshare D200): 0.1–8 m, 360°, 4000 amostras/s, 6 Hz por omissão (2–8 Hz), 667 pontos/volta a 6 Hz,
precisão ±5 mm até 0.5 m e ±1 % de 1 a 6 m (alvo branco), 230400 baud UART, 5 V. Tópico `/scan`, frame `laser_frame`
(eixo X do laser = frente do robô). Parâmetros no topo de `gazebo_control.xacro` (ruído do simulador = 0.01 m, valor médio).
O mundo precisa do plugin `gz-sim-sensors-system` (já incluído em `factory.world`).
No AMCL usar `laser_min_range` ≈ 0.12 (leituras junto ao mínimo são ruidosas). Abaixo de 0.1 m o sensor não mede nada:
não contar com o LIDAR para travar nem para o encaixe final nas estações.
Dimensões 96.3 × 59.8 × 38.8 mm, 101 g. As colisões (caixa única), a massa e a inércia no `robot_core.xacro` / `tools/mass_budget.py` já são as do D200.
Só a mesh visual `rp_lidar.stl` ainda é a do RPLidar A1 (por trocar quando houver o CAD do D200; nessa altura volta a medir-se o eixo, `lidar_x`).

### Pilares
`src/worlds/factory.world` tem 6 pilares (`pilar.stl`, Ø100 mm × 210 mm) nas 6 áreas de marcadores da secção 7 das regras.
**As posições são provisórias**: as regras não dão coordenadas. Os dois cantos ocupados pelos warehouses (sup. esquerdo e inf. direito)
estão recuados em y para não colidirem com eles; o do canto inferior esquerdo fica acima da área de partida (fig. 8.1). Ø100 mm toca nos 4 lados da área de 10 × 10 cm; ver se é aceite ou usar Ø80 mm.

> **Altura do plano:** as paredes das máquinas e armazéns têm 130 mm (figuras 4.2 e 4.3 das regras) e os pilares 210 mm,
> por isso o plano tem de ficar entre os dois. O `resto` foi esticado no CAD (agora 67 mm, topo a 87 mm na placa) e o D200
> assenta em cima. `lidar_z` = `resto_top` + `lidar_plane_h` (0.027 m, **estimativa**: os datasheets não dão a altura do plano)
> => plano a ~135 mm do chão, só ~5 mm acima das paredes. Mede no sensor real a distância da base ao meio da janela ótica
> e atualiza `lidar_plane_h`; se ficar abaixo de ~140 mm do chão, sobe o LIDAR (espaçador ou `resto` mais alto).

## Por fazer
primeiro eu quero deixar tudo da parte "fisica" do robo bem, depois vou querer mudar o mapa porque eu vou fazer amcl, o mapa vai ser aquele que está nas regras da competição + uns pilares nos locais indicados, e so depois vou fazer eu o algoritmo em python para reagir aos valores dados do lidar, não esquecer de configurar este lidar de acordo com este link: (link em falta)