#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class PositionPredictor:
    def __init__(self):
        self.delay = 0.0
        self.dt = 0.0
        self.counter = 0
        self.cmd_vel = np.array([0.0, 0.0, 0.0])

        self.odom_sub = rospy.Subscriber("/odometry", Odometry, self.callback_odometry)
        self.cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel", TwistStamped, self.callback_cmd_vel
        )
        self.est_odom_pub = rospy.Publisher(
            "/estimated_odometry", Odometry, queue_size=10
        )

    def callback_odometry(self, msg):
        self.estimated_odometry = Odometry()
        recv_time = rospy.Time.now()
        pub_time = msg.header.stamp
        self.delay = (recv_time - pub_time).to_sec()
        self.delay = 0
        if self.counter < 500:  # sliding window of ~5s
            self.counter = self.counter + 1
        self.dt = np.divide(self.delay + (self.counter - 1) * self.dt, self.counter)
        self.estimated_odometry = msg
        self.estimated_odometry.pose.pose.position.x = (
            msg.pose.pose.position.x + self.cmd_vel[0] * self.dt
        )
        self.estimated_odometry.pose.pose.position.y = (
            msg.pose.pose.position.y + self.cmd_vel[1] * self.dt
        )
        self.estimated_odometry.pose.pose.position.z = (
            msg.pose.pose.position.z + self.cmd_vel[2] * self.dt
        )
        self.estimated_odometry.header.stamp = rospy.Time.now()
        self.est_odom_pub.publish(self.estimated_odometry)

    def callback_cmd_vel(self, msg):
        self.cmd_vel[0] = msg.twist.linear.x
        self.cmd_vel[1] = msg.twist.linear.y
        self.cmd_vel[2] = msg.twist.linear.z


if __name__ == "__main__":
    rospy.init_node("position_prediction_node", anonymous=True)
    node = PositionPredictor()
    rospy.spin()
