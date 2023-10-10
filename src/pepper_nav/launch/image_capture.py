#!/usr/bin/python

import os
import sys
import time
from naoqi import ALProxy

def main():
        # Set the IP address of your robot here
        ip = "10.1.1.140"
        port = 9559
        session = ALProxy("ALPhotoCapture", ip, port)

        session.setResolution(2)
        session.setPictureFormat("jpg")
        a = session.takePictures(2,"home/zaclab/Desktop/","image")
        print(a)

if __name__ == "__main__":
        main()