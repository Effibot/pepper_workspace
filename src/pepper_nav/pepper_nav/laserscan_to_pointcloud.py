#! /bin/python3
import math

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import rclpy
import pepper_nav.laser_geometry as lg
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformException


# define helper function to convert euler angles to quaternion
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


# define helper function to multiply quaternions
def quaternion_multiply(q0, q1):
    """
    Multiplies two quaternions.

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0[0]
    x0 = q0[1]
    y0 = q0[2]
    z0 = q0[3]

    # Extract the values from q1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion


class LaserScanPC2(Node):
    def __init__(self):
        super().__init__("laser_pc2_converter")
        # declare the target frame
        self.target_frame = "base_link"
        # declare the source frame
        self.source_frame = "base_footprint"
        # declare the buffer which will hold the transform data
        self.tf_buffer = tf2_ros.Buffer()
        # declare the listener which will listen to the transform data
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # initialize the transform broadcaster which will broadcast the transform data
        # self.tf_broadcaster = TransformBroadcaster(self)
        # declare the timer which will trigger the callback function
        # self.timer = self.create_timer(0.1, self.timer_callback)
        # declare the transform object which will hold the transform data
        self.transform = None
        self.trasl, self.rot = None, None
        # declare the laser projection object which will convert the laser scan data to point cloud data
        self.lp = lg.LaserProjection()
        # declare the subscriber which will subscribe to the laser scan data
        self.scan_topic = (
            self.declare_parameter("scan_topic", "laser").get_parameter_value().string_value
        )
        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 1)
        # declare the publisher which will publish the point cloud data
        self.pc_topic = f"{self.scan_topic}/pointcloud"
        # declare the publisher which will publish the point cloud data
        self.pc_pub = self.create_publisher(PointCloud2, self.pc_topic, 1)

    def scan_callback(self, msg):
        # try get the transform data
        try:
            self.transform = self.tf_buffer.lookup_transform(
                target_frame=self.target_frame,
                source_frame=self.source_frame,
                time=rclpy.time.Time(),
            )
            pc2_msg = self.lp.projectLaser(msg)
            self.get_logger().info(f"pc2_msg: {pc2_msg}")
            self.get_logger().info(f"Transform: {self.transform}")
            transformed_msg = self.tf_buffer.transform(
                
                self.target_frame,
                timeout=rclpy.time.Duration(seconds=0.1),
            )
            self.get_logger().info(f"Transformed msg: {transformed_msg}")
            ## transform the laser scan data to the target frame
            # transformed_msg =
            ## convert the laser scan data to point cloud data
            # cloud_out = self.lp.projectLaser()
            ## publish the point cloud data
            # self.pc_pub.publish(cloud_out)
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {self.source_frame} to {self.target_frame}: {ex}"
            )
            return


def main():
    rclpy.init()
    node = LaserScanPC2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
