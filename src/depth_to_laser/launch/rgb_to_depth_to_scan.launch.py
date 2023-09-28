from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_dir = FindPackageShare("depth_to_laser")
    # Get the params file for the pointcloud_to_laserscan node
    cloud_to_scan_params = PathJoinSubstitution(
        [pkg_dir, "params", "pc_2_laser.yaml"]
    )
    # Params substitutions for topic remappings
    depth_camera_info_topic = LaunchConfiguration("depth_camera_info_topic")
    depth_image_raw_topic = LaunchConfiguration("depth_image_raw")
    pointcloud_topic_out = LaunchConfiguration("pointcloud_topic_out")
    # Define the arguments for the launch file
    depth_camera_info_topic_arg = DeclareLaunchArgument(
        "depth_camera_info_topic",
        default_value="/camera/depth/camera_info",
    )

    depth_image_raw_arg = DeclareLaunchArgument(
        "depth_image_raw",
        default_value="/camera/depth/image_raw",
    )
    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic_out",
        default_value="/camera/depth/points",
    )
    # Define a composable node to convert depth images to pointclouds

    xyzi_radial_node = ComposableNode(
        package="depth_image_proc",
        plugin="depth_image_proc::PointCloudXyzNode",
        name="point_cloud_xyz_node",
        remappings=[
            ("camera_info", depth_camera_info_topic),
            ("image_raw", depth_image_raw_topic),
            ("image", '/camera/depth/converted_image'),
        ],
    )

    # Launch a ContainerNode with the above node definition

    container = ComposableNodeContainer(
        name="container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[xyzi_radial_node],
        output="screen",
    )

    # Launch the pointcloud_to_laserscan node

    pc_to_scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        parameters=[cloud_to_scan_params],
        remappings=[("cloud_in", pointcloud_topic_out), ("scan", "/scan")],
    )

    return LaunchDescription(
        [
            depth_camera_info_topic_arg,
            depth_image_raw_arg,
            pointcloud_topic_arg,
            container,
            pc_to_scan_node,
        ]
    )
