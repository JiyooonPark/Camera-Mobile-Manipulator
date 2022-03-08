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
        self.subscriber_odom = rospy.Subscriber("/cmm/darknet_ros", String, self.callback_next)
        self.i = 0
        self.next_cmd_vel= Twist()
        self.not_found = 0

    def callback_next(self, msg):

        if msg.data != "0/0":
            self.not_found = 0
            print("Updated")
        
            x, size = msg.data.split("/")
            x = float(x)
            size = float(size)

            # if x > 0.6:
            #     self.next_cmd_vel.angular.z = -0.07
            # elif 0.6 >= x >= 0.4:
            #     self.next_cmd_vel.angular.z = 0
            # else:
            #     self.next_cmd_vel.angular.z = 0.07

            if x > 0.5:
                self.next_cmd_vel.angular.z = -0.4*abs(0.5-x)
            else:
                self.next_cmd_vel.angular.z = 0.4*abs(0.5-x)
              
            if size>0.3:
                self.next_cmd_vel.linear.x = -0.05
            elif 0.3>=size>=0.2:
                self.next_cmd_vel.linear.x = 0
            else:
                self.next_cmd_vel.linear.x = 0.05
        
        else:
            self.not_found+=1

            if self.not_found == 1:
                self.next_cmd_vel.angular.z *= 2

        self.publisher_cmd_vel.publish(self.next_cmd_vel)
        print(self.next_cmd_vel)

    def move(self):
        while True:
            self.publisher_cmd_vel.publish(self.next_cmd_vel)
    def move2(self):
        self.next_cmd_vel = Twist()
        self.next_cmd_vel.linear.y = 0.1
        while True:
            self.publisher_cmd_vel.publish(self.next_cmd_vel)
        
if __name__ == '__main__':

    rospy.init_node('RIDGEBACK', anonymous=True)
    rospy.Rate(100)
    Rid = Ridgeback()
    rospy.sleep(2)
    Rid.move()
    rospy.spin()