#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class VelocityController:
    def __init__(self):
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.des_position = np.array([0.0, 0.0, 0.0])
        self.des_orientation = np.array([0.0, 0.0, 0.0, 1.0])

        self.odom_sub = rospy.Subscriber(
            "/estimated_odometry", Odometry, self.callback_odometry
        )
        self.setpoint_sub = rospy.Subscriber(
            "/setpoint_position", Odometry, self.callback_setpoint
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", TwistStamped, queue_size=10)

    def callback_odometry(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w
        self.velocity_publisher()

    def callback_setpoint(self, msg):
        self.des_position[0] = msg.pose.pose.position.x
        self.des_position[1] = msg.pose.pose.position.y
        self.des_position[2] = msg.pose.pose.position.z
        self.des_orientation[0] = msg.pose.pose.orientation.x
        self.des_orientation[1] = msg.pose.pose.orientation.y
        self.des_orientation[2] = msg.pose.pose.orientation.z
        self.des_orientation[3] = msg.pose.pose.orientation.w

    def velocity_publisher(self):
        ## INSERT YOUR CODE HERE
        pass  # remove pass after inserting your code


if __name__ == "__main__":
    rospy.init_node("velocity_controller_node", anonymous=True)
    node = VelocityController()
    rospy.spin()
