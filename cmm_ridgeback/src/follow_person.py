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
        self.subscriber_darknet = rospy.Subscriber("/cmm/darknet_ros", String, self.callback_darknet)
        self.linear_speed = 0.1
        self.reached = False
        self.i = 0
        self.x_ranges = [0.7, 0.55, 0.45, 0.3]
        self.size_ranges = [0.3, 0.2, 0.1]
        self.cmd_vel_msg = Twist()
        self.move_status="0.5/0.5/0.2"

    def callback_darknet(self, msg):
        
        self.dkn_x, self.dkn_y, self.dkn_size = msg.data.split("/")
        self.dkn_x = float(self.dkn_x)
        self.dkn_y = float(self.dkn_y)
        self.dkn_size = float(self.dkn_size)

        move_status = ""

        if self.dkn_x >self.x_ranges[0]:
            move_status+="right2/"
        elif self.x_ranges[0]>=self.dkn_x>self.x_ranges[1]:
            move_status+="right/"
        elif self.x_ranges[1]>=self.dkn_x>self.x_ranges[2]:
            move_status+="straight/"
        elif self.x_ranges[2]>=self.dkn_x>self.x_ranges[3]:
            move_status+="left/"
        elif self.x_ranges[3]>=self.dkn_x:
            move_status+="left2/"
        elif self.dkn_x == 0:
            move_status += self.move_status[0]+'/'
        else:
            move_status += self.move_status[0]+'/'
            
        if self.dkn_size > self.size_ranges[0]:
            move_status+="back/"
        elif self.size_ranges[0] >= self.dkn_size > self.size_ranges[1]:
            move_status+="stay/"
        elif self.size_ranges[1] >= self.dkn_size > self.size_ranges[2]:
            move_status+="forward/"
        elif self.size_ranges[2] >= self.dkn_size > 0:
            move_status+="forward/"
        elif self.dkn_size == 0:
            move_status += self.move_status[1]+'/'
        else:
            move_status += self.move_status[1]+'/'

        self.move_status = move_status.split('/')
        
        print(self.move_status)

        if self.move_status[0] =="right2":
            self.relative_rotate(-4)
        elif self.move_status[0] =="right":
            self.relative_rotate(-1) 
        elif self.move_status[0] =="straight":
            self.relative_rotate(0)     
        elif self.move_status[0] =="left":
            self.relative_rotate(1)   
        elif self.move_status[0] =="left2":
            self.relative_rotate(4)   
        
        if self.move_status[1] =="back":
            self.relative_move(-0.5, 0)
        elif self.move_status[1] =="stay":
            self.relative_move(0, 0) 
        elif self.move_status[1] =="forward":
            self.relative_move(0.6, 0)  

        self.move()

    def callback_odom(self, msg):

        # get position
        pose_position = msg.pose.pose.position
        self.position_x = pose_position.x
        self.position_y = pose_position.y

        # get orientation
        self.orientation_q = msg.pose.pose.orientation
        orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]

        (roll, pitch, self.yaw) = self.euler_from_quaternion(orientation_list)
        if self.yaw<0:
            angle = -self.yaw*180/math.pi
        else:
            angle = 360-self.yaw*180/math.pi

        self.rad = math.radians(angle)


    def relative_move(self, x, y):
        divide = 4
        
        self.cmd_vel_msg.linear.x = x/divide
        self.cmd_vel_msg.linear.y = y/divide

    def calculate_distance(self, r_goal):
        return math.sqrt(r_goal[0]**2+ r_goal[1]**2)


    def relative_rotate(self, target_angle):
        
        z = 0.1 *target_angle

        self.cmd_vel_msg.angular.z= z

    def move(self):
        seconds = time.time()
        while time.time() - seconds < 0.05:
            self.publisher_cmd_vel.publish(self.cmd_vel_msg)
        
if __name__ == '__main__':

    rospy.init_node('RIDGEBACK', anonymous=True)

    Rid = Ridgeback()
    # Rid.relative_rotate(10)
    # Rid.relative_rotate(-10)
    # rospy.sleep(2)
    rospy.spin()