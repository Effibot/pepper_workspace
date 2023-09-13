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
    pkg_dir = get_package_share_directory("depth_to_laser")

    ptl_params_args = DeclareLaunchArgument(
        "pointcloud_to_laserscan_params_file",
        default_value=os.path.join(
            pkg_dir, "params/pc_2_laser.yaml"
        ),
    )
    pointcloud_to_laserscan_params_file = LaunchConfiguration(
        "pointcloud_to_laserscan_params_file"
    )

    # The component container that several ComposableNodes can run in, in the same process
    cc_arg = DeclareLaunchArgument(
        "component_container_name", default_value="DepthLaserComponentContainer"
    )
    cc_name = LaunchConfiguration("component_container_name")
    component_container = Node(
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        name=cc_name,
    )

    # Launch everything with perception from rgbd_launch
    depth_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_dir, "launch"), "/depth_perception.launch.py"]
        ),
        launch_arguments={"container": cc_name}.items(),
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        parameters=[pointcloud_to_laserscan_params_file],
        remappings=[("cloud_in", "/depth_camera/points")],
    )

    return LaunchDescription(
        [
            ptl_params_args,
            cc_arg,
            component_container,
            depth_perception,
            pointcloud_to_laserscan,
        ]
    )
