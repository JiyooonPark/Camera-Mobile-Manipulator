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
        self.subscriber_turn = rospy.Subscriber('/cmm/turn', String, self.callback_turn)
        self.subscriber_odom = rospy.Subscriber("/cmm/darknet_ros", String, self.callback_next)
        self.i = 0
        self.next_cmd_vel= Twist()
        self.not_found = 0
        self.start_time = time.time()
        self.turn = "rid"
    def callback_turn(self, msg):
        self.turn = msg.data
        # print(self.turn)

    def callback_next(self, msg):
        if self.turn=="iiwa":
            self.next_cmd_vel.angular.z = 0
            self.next_cmd_vel.linear.x = 0
            self.next_cmd_vel.linear.y = 0

        else:
            if msg.data != "0/0":
                self.not_found = 0
                # print("Updated")
            
                x, size = msg.data.split("/")
                x = float(x)
                size = float(size)


                if x > 0.6:
                    self.next_cmd_vel.angular.z = -0.9*abs(0.5-x)
                elif 0.6 >= x > 0.55:
                    self.next_cmd_vel.angular.z = -0.3*abs(0.5-x)
                elif 0.55 >= x > 0.45:
                    self.next_cmd_vel.angular.z = 0
                elif 0.45 >= x > 0.4:
                    self.next_cmd_vel.angular.z = 0.3*abs(0.5-x)
                else:
                    self.next_cmd_vel.angular.z = 0.9*abs(0.5-x)
                
                if size>=0.85:
                    return
                elif 0.85>size>0.45:
                    self.next_cmd_vel.linear.x = -0.2*(1-size)
                elif 0.45>=size>=0.3:
                    self.next_cmd_vel.linear.x = -0.1*(1-size)
                elif 0.3>=size>=0.25:
                    self.next_cmd_vel.linear.x = 0
                elif 0.25>=size>=0.15:
                    self.next_cmd_vel.linear.x = 0.1*(1-size)
                else:
                    self.next_cmd_vel.linear.x = 0.2*(1-size)
            
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
            self.publisher_turn.publish("No")
            if time.time() - self.start_time > 15:
                print("Stopping driving")
                for i in range(10):
                    self.publisher_turn.publish("Yes")
                break
        
if __name__ == '__main__':

    rospy.init_node('RIDGEBACK', anonymous=True)
    rospy.Rate(100)
    Rid = Ridgeback()
    # rospy.sleep(2)
    Rid.move()
    rospy.spin()