import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_bot')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # O Gazebo converte package://my_bot/... em model://my_bot/... e procura em
    # GZ_SIM_RESOURCE_PATH; apontamos para a pasta 'share' que contem my_bot/
    resource_path = os.pathsep.join(
        [os.path.dirname(pkg), os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    set_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    # robot_state_publisher com sim time
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    # Gazebo Harmonic com o mundo do pacote
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + os.path.join(pkg, 'worlds', 'empty.world')}.items())

    # Cria o robo a partir do topico robot_description
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.05'],
        output='screen')

    # Ponte Gazebo <-> ROS 2
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen')

    return LaunchDescription([set_resource_path, rsp, gz, spawn, bridge])