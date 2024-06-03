#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class PositionPredictor:
    def __init__(self):
        self.cmd_vel = np.array([0.0, 0.0, 0.0])

        self.odom_sub = rospy.Subscriber("/odometry", Odometry, self.callback_odometry)
        self.cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel", TwistStamped, self.callback_cmd_vel
        )
        self.est_odom_pub = rospy.Publisher(
            "/estimated_odometry", Odometry, queue_size=10
        )

    def callback_odometry(self, msg):
        ## INSERT YOUR CODE HERE

    def callback_cmd_vel(self, msg):
        self.cmd_vel[0] = msg.twist.linear.x
        self.cmd_vel[1] = msg.twist.linear.y
        self.cmd_vel[2] = msg.twist.linear.z


if __name__ == "__main__":
    rospy.init_node("position_prediction_node", anonymous=True)
    node = PositionPredictor()
    rospy.spin()
