from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # find package directory
    pkg_dir = get_package_share_directory("ros_naoqi_motion")
    # Declare the launch arguments
    ip_declare = DeclareLaunchArgument(
        "host_ip", default_value="localhost", description="Ip address of host"
    )
    host_ip = LaunchConfiguration("host_ip")
    port_declare = DeclareLaunchArgument(
        "host_port",
        default_value="9999",
        description="Port to be used for the connection",
    )
    host_port = LaunchConfiguration("host_port", default=9999)
    nao_ip_declare = DeclareLaunchArgument(
        "nao_ip", default_value="127.0.0.1", description="Ip address of the robot"
    )
    nao_ip = LaunchConfiguration("nao_ip", default="127.0.0.1")
    nao_port_declare = DeclareLaunchArgument(
        "nao_port",
        default_value="9559",
        description="Port to be used for the connection",
    )
    nao_port = LaunchConfiguration("nao_port", default=9559)

    enc_declare = DeclareLaunchArgument(
        "encoding",
        default_value="utf-8",
        description="Encoding to be used for the connection",
    )
    encoding = LaunchConfiguration("encoding", default="utf-8")

    # Declare the launch nodes
    motion_server_node = Node(
        package="ros_naoqi_motion",
        executable="naoqi_motion",
        name="naoqi_motion",
        output="screen",
        parameters=[
            {"host_ip": host_ip},
            {"host_port": host_port},
            {"encoding": encoding},
            {"nao_ip": nao_ip},
            {"nao_port": nao_port},
        ],
    )

    # Create the launch description
    ld = LaunchDescription(
        [
            ip_declare,
            port_declare,
            enc_declare,
            nao_ip_declare,
            nao_port_declare,
            motion_server_node,
        ]
    )

    return ld
