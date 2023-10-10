#! /usr/bin/python3

import os
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
import launch.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

global ip, wake_up_script, pkg_dir, to_viz
pkg_dir = get_package_share_directory("naoqi_driver")
wake_up_script = os.path.join(pkg_dir, "launch", "wake_up.py")
to_viz = True


def robot_wake_up(context, *args, **kwargs):
    ip = LaunchConfiguration("nao_ip").perform(context)
    # wake up the robot at runtime
    done = subprocess.run(["python2", wake_up_script, f"--ip={str(ip)}"])


def generate_launch_description():
    # declare ip argument
    ip_declare = DeclareLaunchArgument(
        "nao_ip", default_value="127.0.0.1", description="Ip address of the robot"
    )
    nao_ip = LaunchConfiguration("nao_ip")
    get_ip = OpaqueFunction(function=robot_wake_up)
    # declare other arguments
    nao_port = DeclareLaunchArgument(
        "nao_port",
        default_value="9559",
        description="Port to be used for the connection",
    )
    username = DeclareLaunchArgument(
        "username",
        default_value="nao",
        description="Username for the connection",
    )
    pwd = DeclareLaunchArgument(
        "password",
        default_value="no_password",
        description="Password for the connection",
    )
    net_int = DeclareLaunchArgument(
        "network_interface",
        default_value="eth0",
        description="Network interface to be used",
    )
    namespace = DeclareLaunchArgument(
        "namespace",
        default_value="naoqi_driver",
        description="Name of the namespace to be used",
    )

    # generate RVIZ2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_dir, "share", "pepper.rviz")],
    )

    # generate naoqi_driver node
    naoqi_node = Node(
        package="naoqi_driver",
        executable="naoqi_driver_node",
        name=[launch.substitutions.LaunchConfiguration("namespace")],
        parameters=[
            {
                "nao_ip": nao_ip,
                "nao_port": launch.substitutions.LaunchConfiguration("nao_port"),
                "password": launch.substitutions.LaunchConfiguration("password"),
                "network_interface": launch.substitutions.LaunchConfiguration(
                    "network_interface"
                ),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            ip_declare,
            get_ip,
            nao_port,
            username,
            pwd,
            net_int,
            namespace,
            naoqi_node,
            rviz_node,
        ]
    )
