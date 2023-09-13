import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import LoadComposableNodes
from launch.conditions import (
    LaunchConfigurationNotEquals,
    LaunchConfigurationEquals,
)
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rgbd_dir = get_package_share_directory("rgbd_launch")

    cc_arg = DeclareLaunchArgument(
        "container", default_value="ComponentContainer"
    )
    cc = LaunchConfiguration("container")
    depth_arg = DeclareLaunchArgument("depth", default_value="depth")
    depth_prefix_arg = DeclareLaunchArgument(
        "depth_camera_prefix", default_value="camera/depth"
    )
    depth_prefix = LaunchConfiguration("depth_camera_prefix")
    depth_ns = LaunchConfiguration("depth")
    rgb_arg = DeclareLaunchArgument("rgb", default_value="rgb")
    rgb_prefix_arg = DeclareLaunchArgument(
        "rgb_camera_prefix", default_value="camera/front"
    )
    rgb_prefix = LaunchConfiguration("rgb_camera_prefix")
    rgb_ns = LaunchConfiguration("rgb")

    depth_registered = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(rgbd_dir, "launch", "includes"),
                "/depth_registered.launch.py",
            ]
        ),
        launch_arguments={"container": cc}.items(),
    )
    depth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(rgbd_dir, "launch", "includes"), "/depth.launch.py"]
        ),
        launch_arguments={
            "container": cc,
            "depth": depth_ns,
            "camera_topic_prefix": depth_prefix,
        }.items(),
    )

    rgb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(rgbd_dir, "launch", "includes"), "/rgb.launch.py"]
        ),
        launch_arguments={
            "container": cc,
            "rgb": rgb_ns,
            "camera_topic_prefix": rgb_prefix,
        }.items(),
    )

    return LaunchDescription(
        [
            cc_arg,
            depth_arg,
            rgb_arg,
            rgb_prefix_arg,
            depth_prefix_arg,
            depth,
            rgb,
            depth_registered,
        ]
    )
