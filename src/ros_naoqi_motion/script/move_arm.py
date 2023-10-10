#! /usr/bin/python2
# -*- coding: utf-8 -*-
import argparse
import sys
from time import sleep

import qi


MOVE_ARM_DOWN = "move_arm_down"

class MotionCore:
    def __init__(self):
        self.isConnected = False
        self.session = None
        self.motion_service = None
        self.motion_service_name = "ALMotion"
        self.posture_service = None
        self.posture_service_name = "ALRobotPosture"
        self.frame = None
        self.right_chain = [
            "RShoulderPitch",
            "RShoulderRoll",
            "RElbowYaw",
            "RElbowRoll",
            "RWristYaw",
            "RHand"
        ]
        self.is_running = False

    def connect(self, ip, port):
        self.session = qi.Session()
        try:
            self.session.connect("tcp://" + ip + ":" + str(port))
            self.isConnected = True
            self.motion_service = self.session.service(self.motion_service_name)
            self.posture_service = self.session.service(self.posture_service_name)
            print("Connected to Pepper")
        except RuntimeError:
            print(
                "Can't connect to Naoqi at ip \""
                + ip
                + '" on port '
                + str(port)
                + ".\n"
                "Please check your script arguments. Run with -h option for help."
            )
            sys.exit(1)



    def move_it(self):
        if self.motion_service is None or self.posture_service is None:
            print("Not connected to Pepper")
            return

        if self.motion_service.robotIsWakeUp() != 1:
            # Wake up robot
            self.motion_service.wakeUp()

        if self.posture_service.getPosture() != "StandInit":
            # Send robot to Stand Init
            self.posture_service.goToPosture("StandInit", 0.5)

        self.is_running = True
        try:
            while True:
                self.move_R_arm()
        except KeyboardInterrupt:
            print("\nKeyboard Interrupt (SIGINT)")
            self.is_running = False
            sys.exit(0)


    def move_R_arm(self):
        assert self.motion_service is not None
        angle_list = [-0.1, -0.6, -0.18, 0.8, 0.1, 0.1]
        speed_list = [1.9, 1.9, 0.9, 0.9, 0.1, 0.5]
        self.motion_service.angleInterpolation(
            self.right_chain, angle_list, speed_list, True
        )


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    # IP address of the robot
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="Robot IP address")
    # Port number of the robot
    parser.add_argument("--port", type=str, default=9559, help="Robot port number")
    # Language
    parser.add_argument(
        "--language", type=str, default="Italian", help="Robot language"
    )
    # Text to say
    parser.add_argument("--text", type=str, default="Hello, world!", help="Text to say")
    # Encoding
    parser.add_argument("--encoding", type=str, default="utf-8", help="Text encoding")
    # Parse arguments
    args = parser.parse_args()
    # Create a MotionCore object and connect to Pepper
    core = MotionCore()
    try:
        core.connect(args.ip, args.port)
        # move arm
        core.move_it()
    except RuntimeError, e:
        print("Can't connect to Pepper\n" + str(e))
        sys.exit(1)
