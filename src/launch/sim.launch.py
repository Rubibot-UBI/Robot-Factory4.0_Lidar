import math
import os
import random
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start_gazebo(context, *args, **kwargs):
    """Gera o mundo (factory.world + objetos aleatorios fora da pista) e arranca o Gazebo com ele.

    ros2 launch my_bot sim.launch.py                    -> seed aleatoria (impressa no terminal)
    ros2 launch my_bot sim.launch.py seed:=42           -> mapa reproduzivel
    ros2 launch my_bot sim.launch.py seed:=42 object_counts:=pilar=3,cone=0,pessoa=2
    ros2 launch my_bot sim.launch.py extra_objects:=false   -> so o mundo base das regras
    """
    pkg = get_package_share_directory('my_bot')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    base_world = os.path.join(pkg, 'worlds', 'factory.world')

    msgs = []
    world = base_world
    if LaunchConfiguration('extra_objects').perform(context).lower() in ('true', '1', 'yes'):
        sys.path.insert(0, os.path.join(pkg, 'worlds'))
        import world_generator
        res = world_generator.generate_world(
            seed=LaunchConfiguration('seed').perform(context),
            base=base_world,
            counts=world_generator.parse_counts(LaunchConfiguration('object_counts').perform(context)))
        world = res['world']
        kinds = {}
        for o in res['objects']:
            kinds[o['type']] = kinds.get(o['type'], 0) + 1
        msgs.append(LogInfo(msg='[mapa] seed=%d (repetir com seed:=%d) | %d objetos extra: %s\n'
                                '[mapa] mundo: %s | verdade-terreno: %s'
                            % (res['seed'], res['seed'], len(res['objects']),
                               ', '.join('%s=%d' % kv for kv in kinds.items()),
                               res['world'], res['manifest'])))

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world}.items())
    return msgs + [gz]

# Area de partida (verde, fig. 8.1 das regras): quadrado de 0.35 x 0.35 m no canto inferior esquerdo da arena
# (chao 1.7 x 1.2 m, origem no centro, y = norte). O canto superior direito da area coincide com o canto
# inferior esquerdo da maquina A -> x in [-0.85, -0.50], y in [-0.60, -0.25].
START_CX, START_CY = -0.675, -0.425  # centro da area de partida
START_JITTER_X = 0.07                # +- m (meia largura da area = 0.175; o robo tem meia diagonal ~0.06)
START_JITTER_Y = 0.07                # +- m (idem em y)
START_JITTER_YAW = 0.17              # +- rad (~10 graus) em torno de norte


def _spawn_robot(context, *args, **kwargs):
    """Spawn aleatorio perto do centro da area de partida, a apontar para norte (+Y) com pequeno desvio.

    Com seed:=N a pose e reproduzivel (usa N+1, para nao ficar ligada a seed dos objetos).
    A frente do robo e +X em base_link, logo yaw = +pi/2 = norte.
    """
    seed = LaunchConfiguration('seed').perform(context)
    rng = random.Random(int(seed) + 1) if seed.strip() else random.Random()
    x = START_CX + rng.uniform(-START_JITTER_X, START_JITTER_X)
    y = START_CY + rng.uniform(-START_JITTER_Y, START_JITTER_Y)
    yaw = math.pi / 2 + rng.uniform(-START_JITTER_YAW, START_JITTER_YAW)

    spawn = Node(
        package='ros_gz_sim', executable='create',
        # base_link esta a altura do eixo (raio da roda = 0.028 m); 0.03 m = pousa quase sem queda
        arguments=['-topic', 'robot_description', '-name', 'my_bot',
                   '-x', f'{x:.4f}', '-y', f'{y:.4f}', '-z', '0.03',
                   '-Y', f'{yaw:.4f}'],
        output='screen')
    info = LogInfo(msg='[spawn] x=%.3f y=%.3f yaw=%.1f graus (verdade-terreno da pose inicial)'
                       % (x, y, math.degrees(yaw)))
    return [info, spawn]


def generate_launch_description():
    pkg = get_package_share_directory('my_bot')

    # O Gazebo converte package://my_bot/... em model://my_bot/... e procura em
    # GZ_SIM_RESOURCE_PATH; apontamos para a pasta 'share' que contem my_bot/
    resource_path = os.pathsep.join(
        [os.path.dirname(pkg), os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    set_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    # robot_state_publisher com sim time
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    # Gazebo Harmonic: o mundo e gerado a cada arranque (ver _start_gazebo)
    args = [
        DeclareLaunchArgument('seed', default_value='',
                              description='seed do mapa (inteiro); vazio = aleatoria'),
        DeclareLaunchArgument('object_counts', default_value='',
                              description='forcar quantidades, ex.: pilar=3,cone=0 '
                                          '(tipos: pilar,pessoa,cadeira,mesa,quadrado,cone,balde)'),
        DeclareLaunchArgument('extra_objects', default_value='true',
                              description='false = so o mundo base, sem objetos fora da pista'),
    ]
    gz = OpaqueFunction(function=_start_gazebo)

    # Cria o robo a partir do topico robot_description (pose aleatoria na area de partida, ver _spawn_robot)
    spawn = OpaqueFunction(function=_spawn_robot)

    # Ponte Gazebo <-> ROS 2
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen')

    return LaunchDescription(args + [set_resource_path, rsp, gz, spawn, bridge])