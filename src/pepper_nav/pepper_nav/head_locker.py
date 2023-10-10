import rclpy
from rclpy.node import Node
from requests import head
from sensor_msgs.msg import JointState


class HeadLocker(Node):
    def __init__(self):
        super().__init__("head_locker")
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.head_state_callback, 10
        )
        self.default_config = [0.0, 0.0]  # Set the default configuration here
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.filter_fields = [
            "HeadPitch",
            "HeadYaw",
        ]  # Add the fields you want to filter for here
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)

    def head_state_callback(self, msg):
        # get the current pitch and yaw
        self.current_pitch = msg.position[msg.name.index("HeadPitch")]
        self.current_yaw = msg.position[msg.name.index("HeadYaw")]
        # check if the current pitch and yaw are in the default configuration
        if (
            self.current_pitch == self.default_config[0]
            and self.current_yaw == self.default_config[1]
        ):
            # if they are, then we don't need to do anything
            return
        else:
            # if they aren't, then we need to set them back to the default configuration
            self.set_head(self.default_config[0], self.default_config[1])
            return

    def set_head(self, pitch, yaw):
        # generate the message
        msg = JointState(
            name=["HeadPitch", "HeadYaw"],
            position=[pitch, yaw],
            velocity=[0.1, 0.1],
            effort=[0.1, 0.1],
        )
        # publish the message
        self.publisher.publish(msg)
        self.get_logger().info("HeadLocker: Head set to default configuration.")


def main():
    rclpy.init()
    node = HeadLocker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()
