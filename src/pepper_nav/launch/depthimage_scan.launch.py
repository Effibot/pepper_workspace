import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    depth_params_file = os.path.join(get_package_share_directory("pepper_nav"), "params", "depth_to_laser.yaml")
    return LaunchDescription(
        [
            # launch plugin through rclcpp_components container
            ComposableNodeContainer(
                name='container',
                namespace='/camera/depth/',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    # Driver itself
                    # ComposableNode(
                    #    package='depth_image_proc',
                    #    plugin='depth_image_proc::ConvertMetricNode',
                    #    name='convert_metric_node',
                    #    remappings=[('image_raw', '/camera/depth/image_raw'),
                    #                ('camera_info', '/camera/depth/camera_info'),
                    #                ('image', '/camera/depth/converted_image')]
                    # ),
                    ComposableNode(
                        package='depth_image_proc',
                        plugin='depth_image_proc::PointCloudXyzNode',
                        name='point_cloud_xyz_node',
                        remappings=[
                            ('image_rect', '/camera/depth/image_raw'),
                            ('camera_info', '/camera/depth/camera_info'),
                            ('points', '/camera/depth/points'),
                        ],
                    ),
                ],
                output='screen',
            ),
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in', '/camera/depth/points'), ('scan', 'scan')],
                parameters=[depth_params_file],
                name='pointcloud_to_laserscan',
            ),
        ]
    )
