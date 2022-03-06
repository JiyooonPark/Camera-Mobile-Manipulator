#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import math, time
import numpy as np
from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose, Twist
from std_msgs.msg import *

class Ridgeback:

    def __init__(self):

        self.publisher_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # self.subscriber_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odom)
        # self.subscriber_darknet = rospy.Subscriber("/cmm/darknet_ros", String, self.callback_darknet)
        self.linear_speed = 0.1
        self.reached = False
        self.i = 0
        self.cmd_vel_msg = Twist()

    def relative_move(self, x, y):
        divide = 4
        
        self.cmd_vel_msg.linear.x = x/divide
        self.cmd_vel_msg.linear.y = y/divide

    def calculate_distance(self, r_goal):
        return math.sqrt(r_goal[0]**2+ r_goal[1]**2)


    def relative_rotate(self, target_angle):
        
        z = 0.1 *target_angle

        self.cmd_vel_msg.angular.z= z

    def round_rotate(self):
        
        while True:
            print("rotating")
            # self.cmd_vel_msg.linear.x = 0.15
            self.cmd_vel_msg.linear.y = 0.1
            self.cmd_vel_msg.angular.z= -0.05
            self.publisher_cmd_vel.publish(self.cmd_vel_msg)

    def circular_rotate(self):
        
        while True:
            print("rotating")
            # self.cmd_vel_msg.linear.x = 0.05
            # self.cmd_vel_msg.linear.y = 0.05
            self.cmd_vel_msg.angular.z= 0.5
            self.publisher_cmd_vel.publish(self.cmd_vel_msg)

    def move(self):
        seconds = time.time()
        while time.time() - seconds < 0.05:
            self.publisher_cmd_vel.publish(self.cmd_vel_msg)
        
if __name__ == '__main__':

    rospy.init_node('RIDGEBACK', anonymous=True)
    rospy.Rate(10)

    Rid = Ridgeback()
    # Rid.relative_rotate(10)
    # Rid.relative_rotate(-10)
    rospy.sleep(2)
    Rid.round_rotate()
    # rospy.spin()