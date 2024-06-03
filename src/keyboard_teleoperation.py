#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from pynput import keyboard
from tf.transformations import *


class DroneTeleoperator:
    def __init__(self):
        self.pub = rospy.Publisher("/setpoint_position", Odometry, queue_size=10)
        self.odom = Odometry()
        self.odom.header.frame_id = "base_link"

        # Set initial positions
        self.odom.pose.pose.position.x = 0.0
        self.odom.pose.pose.position.y = 0.0
        self.odom.pose.pose.position.z = 0.0
        self.yaw_angle = 0.0
        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = 0.0
        self.odom.pose.pose.orientation.w = 1.0

        # Start keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        ## INSERT YOUR CODE HERE


if __name__ == "__main__":
    rospy.init_node("keyboard_teleoperation_node", anonymous=True)
    node = DroneTeleoperator()
    rospy.spin()
