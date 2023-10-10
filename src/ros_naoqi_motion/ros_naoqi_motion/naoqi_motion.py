# Desc: Web server for Naoqi Motion

#!/usr/bin/python


import os
import signal
import socket
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String as StringMsg

global MOVE_ARM_UP, MOVE_ARM_DOWN
MOVE_ARM_UP = "move_arm_up"
MOVE_ARM_DOWN = "move_arm_down"

move_up_msg = StringMsg(data=MOVE_ARM_UP)
move_down_msg = StringMsg(data=MOVE_ARM_DOWN)

import threading


class WebServer:
    def __init__(
        self, host_ip: str, host_port: int, encoding: str, motion_node: Node
    ) -> None:
        # Constants
        self.host_ip = host_ip
        self.host_port = host_port
        self.encoding = encoding
        # Status
        self.is_closed = True
        # Server instance
        self.server_socket = None
        self.client = None
        # Node instance
        self.node = motion_node

        self.thread = threading.Thread(target=self.start_server)
        self.thread.daemon = True
        self.thread.start()

    def create_server(self):
        # create the server socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # set the socket option to reuse the address
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # bind the socket to the host and port
        self.server_socket.bind((self.host_ip, self.host_port))
        # set total number of clients
        self.server_socket.listen(1)
        return self.server_socket

    def accept_client(self, server):
        # accept new connection
        self.client, address = server.accept()
        self.node.logger.info(f"Accepted connection from {address}")  # type: ignore
        return self.client

    def close_connection(self, server, client):
        # close the connection if it is open
        if not self.get_status():
            self.set_status(False)
            self.node.logger.info("Closing connection")  # type: ignore
            assert isinstance(self.client, socket.socket)
            client.close()
            server.close()
            self.node.logger.info("Connection closed")  # type: ignore
        else:
            self.node.logger.info("Connection already closed")  # type: ignore

    def do_work(self, server, client):
        # checks if the client sended all the data

        while True:
            data = client.recv(4096).decode(encoding=self.encoding).strip()
            # if data is not empty
            if data:
                self.node.logger.info(f"Received: {data}")  # type: ignore
                client.send("ACK\n".encode(self.encoding))
                # checks if the arm should be moved up or down
                if data == MOVE_ARM_UP:
                    self.forward_message(move_up_msg)
                elif data == MOVE_ARM_DOWN:
                    self.forward_message(move_down_msg)
            else:
                self.node.logger.info("Received all data")  # type: ignore
                client.send("JOB_DONE\n".encode(self.encoding))
                self.node.logger.info("Sent: JOB_DONE")  # type: ignore
                self.close_connection(server, client)
                break

    def connection_handler(self):
        try:
            self.node.logger.info(  # type: ignore
                f"Waiting for connection on {self.host_ip}:{self.host_port} [TCP]"
            )
            while True:
                server = self.create_server()
                if self.get_status():
                    client = self.accept_client(server)
                    self.set_status(False)
                    self.do_work(server, client)
                    self.set_status(True)
                else:
                    self.node.logger.info(  # type: ignore
                        f"Waiting for connection on {self.host_ip}:{self.host_port} [TCP]"
                    )
                    time.sleep(1)
        except KeyboardInterrupt:
            self.node.logger.info("Keyboard Interrupt (SIGINT)")  # type: ignore
        finally:
            self.close_connection(self.server_socket, self.client)

    def start_server(self):
        self.connection_handler()

    def set_status(self, status: bool):
        self.is_closed = status

    def get_status(self):
        return self.is_closed

    def forward_message(self, msg):
        self.node.logger.info("received message")  # log message # type: ignore
        self.node.publisher.publish(msg)  # publish to motion_node # type: ignore
        self.node.logger.info("published message")  # log message # type: ignore


class MotionServerNode(Node):
    def __init__(self):
        super().__init__("motion_server_node")  # type: ignore
        self.logger = rclpy.logging.get_logger("motion_server_node")  # type: ignore
        # declare params
        self.declare_parameter(
            "host_ip",
            "localhost",
            ParameterDescriptor(
                description="Ip address of the machine that executes the server, default:localhost"
            ),
        )
        self.declare_parameter(
            "host_port",
            9999,
            ParameterDescriptor(description="Port of the server, default: 9999"),
        )
        self.declare_parameter(
            "encoding",
            "utf-8",
            ParameterDescriptor(description="Encoding to use, default: utf-8"),
        )
        self.declare_parameter(
            "nao_ip",
            "127.0.0.1",
            ParameterDescriptor(
                description="Ip address of the robot, default: localhost"
            ),
        )
        self.declare_parameter(
            "nao_port",
            9559,
            ParameterDescriptor(description="Port of the robot, default: 9559"),
        )

        self.host_ip = self.get_parameter("host_ip").get_parameter_value().string_value
        self.host_port = (
            self.get_parameter("host_port").get_parameter_value().integer_value
        )
        self.encoding = (
            self.get_parameter("encoding").get_parameter_value().string_value
        )
        self.nao_ip = self.get_parameter("nao_ip").get_parameter_value().string_value
        self.nao_port = (
            self.get_parameter("nao_port").get_parameter_value().integer_value
        )

        # declare publisher for motion_server
        self.publisher = self.create_publisher(StringMsg, "motion_server", 10)
        # declare subscriber for motion_server
        self.create_subscription(
            StringMsg, "motion_server", self.motion_server_callback, 10
        )
        # locate the script
        self.motion_core_script = os.path.join(
            get_package_share_directory("ros_naoqi_motion"),
            "script",
            "move_arm.py",
        )

        self.motion_daemon = None
        # create the server
        self.server = WebServer(self.host_ip, self.host_port, self.encoding, self)

    def motion_server_callback(self, msg: StringMsg) -> None:
        self.logger.info("subscriber: received message")
        # parse action
        if msg.data == MOVE_ARM_UP:
            # execute action through the script
            self.logger.info(f"executing action: {msg.data}")
            if self.motion_daemon is not None:
                self.motion_daemon.send_signal(signal.SIGINT)
                self.motion_daemon.kill()
            self.motion_daemon = subprocess.Popen(
                [
                    "python2",
                    self.motion_core_script,
                    f"--ip={self.nao_ip}",
                    f"--port={self.nao_port}",
                ],
            )
            # print(stdout_data)
        elif msg.data == MOVE_ARM_DOWN:
            self.logger.info(f"executing action: {msg.data}")
            if self.motion_daemon is not None:
                self.motion_daemon.send_signal(signal.SIGINT)
                self.motion_daemon.kill()
            else:
                self.logger.info("p is None")


from rclpy.executors import MultiThreadedExecutor


def main():
    rclpy.init()
    node = MotionServerNode()
    print("Node created")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
