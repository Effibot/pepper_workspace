import os
import turtle
import rclpy
from requests import get
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import launch
import launch.actions
from launch import Substitution, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    IncludeLaunchDescription,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # launch config stuffs
    launch_file_dir = os.path.join(
        get_package_share_directory("my_turtle_pkg"), "launch"
    )
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="-2.0")
    y_pose = LaunchConfiguration("y_pose", default="-1.5")

    # Declare model parameter
    model = "model.urdf"

    urdf_path = os.path.join(
        get_package_share_directory("my_turtle_pkg"), "urdf", model
    )

    with open(urdf_path, "r") as infp:
        robot_desc = infp.read()

    # declare which world to use
    world = os.path.join(
        get_package_share_directory("my_turtle_pkg"),
        "world",
        "house.world",
    )

    rclpy.logging._root_logger.info("world: {}".format(world))

    # Gazebo stuffs:

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    # --- #

    # Config state publisher node
    state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_desc, "use_sim_time": use_sim_time},
        ],
    )

    # Spawn turtlebot3 with its service
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    # Load rviz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("my_turtle_pkg"),
                "rviz",
                "config.rviz",
            ),
        ],
    )

    return LaunchDescription(
        [
            gzserver_cmd,
            gzclient_cmd,
            state_publisher_node,
            spawn_turtlebot_cmd,
            # rviz_node,
        ]
    )
