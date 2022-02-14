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

    def euler_from_quaternion(self, orientation_list):

        [x, y, z, w] = orientation_list
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z 

    def callback_darknet(self, msg):
        self.dkn_x, self.dkn_y, self.dkn_size = msg.data.split("/")
        self.dkn_x = float(self.dkn_x)
        self.dkn_y = float(self.dkn_y)
        self.dkn_size = float(self.dkn_size)

        move_status = ""

        if self.dkn_x >self.x_ranges[0]:
            move_status+="right/"
        elif self.x_ranges[0]>=self.dkn_x>self.x_ranges[1]:
            move_status+="right/"
        elif self.x_ranges[1]>=self.dkn_x>self.x_ranges[2]:
            move_status+="straight/"
        elif self.x_ranges[2]>=self.dkn_x>self.x_ranges[3]:
            move_status+="left/"
        elif self.x_ranges[3]>=self.dkn_x:
            move_status+="left/"
        elif self.dkn_x == 0:
            move_status = self.move_status
            
        if self.dkn_size > self.size_ranges[0]:
            move_status+="back/"
        elif self.size_ranges[0] >= self.dkn_size > self.size_ranges[1]:
            move_status+="stay/"
        elif self.size_ranges[1] >= self.dkn_size > self.size_ranges[2]:
            move_status+="forward/"
        elif self.size_ranges[2] >= self.dkn_size > 0:
            move_status+="forward/"
        elif self.dkn_size == 0:
            move_status = self.move_status

        self.move_status = move_status

        print(self.move_status)

        self.move_status = self.move_status.split("/")
        if self.move_status[0] =="right":
            self.relative_rotate(2)
        elif self.move_status[0] =="straight":
            self.relative_rotate(0)     
        elif self.move_status[0] =="left":
            self.relative_rotate(-2)    
        
        if self.move_status[1] =="back":
            self.relative_move(-0.05, 0)
        elif self.move_status[1] =="stay":
            self.relative_move(0, 0) 
        elif self.move_status[1] =="forward":
            self.relative_move(0.05, 0)  

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

    def fixed_goal_time(self, goal_x, goal_y):

        x_move = goal_x-self.position_x
        y_move = goal_y-self.position_y

        goal = np.array([[x_move],[y_move]])

        rotation_matrix = np.array([[math.cos(self.rad), -math.sin(self.rad)], 
                                    [math.sin(self.rad), math.cos(self.rad)]])

        r_goal = np.matmul(rotation_matrix, goal)

        self.relative_move(float(r_goal[0][0]), float(r_goal[1][0]), duration=10)

    def relative_move(self, x, y, duration=0.7):
        
        print("moving to :", x, y)
        publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        cmd = Twist()
        cmd.linear.x = x/duration
        cmd.linear.y = y/duration
        cmd.linear.z = 0

        rospy.sleep(1)

        seconds = time.time()
        while time.time() - seconds < duration:
            publisher.publish(cmd)

    def fixed_goal(self, goal_x, goal_y, duration=5):

        self.publish_state('0')
        # print(f'Executing ridgeback drawing #{self.state}')
        cmd = Twist()

        while not self.reached:

            x_move = goal_x-self.position_x
            y_move = goal_y-self.position_y
            self.i +=1
            if self.i % 300000 == 0:
                print("distance: ", dist)
            goal = np.array([[x_move],[y_move]])

            rotation_matrix = np.array([[math.cos(self.rad), -math.sin(self.rad)], 
                                        [math.sin(self.rad), math.cos(self.rad)]])

            r_goal = np.matmul(rotation_matrix, goal)
            dist = self.calculate_distance(r_goal)
            
            if not self.reached:
                if dist > 0.3:
                    cmd.linear.x = self.linear_speed * r_goal[0][0]
                    cmd.linear.y = self.linear_speed * r_goal[1][0]

                if dist < 0.3:
                    cmd.linear.x = 0
                    cmd.linear.y = 0
                    self.reached = True
                    print('DONE')
            self.publisher_cmd_vel.publish(cmd)
        self.reached = False

    def calculate_distance(self, r_goal):
        return math.sqrt(r_goal[0]**2+ r_goal[1]**2)


    def relative_rotate(self, target_angle):
        
        if target_angle > 0 :
            z = -0.5
            print("moving right")
        else:
            z = 0.5
            print("moving left")
        cmd = Twist()
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = z
        seconds = time.time()
        while time.time() - seconds < target_angle*0.2:
            self.publisher_cmd_vel.publish(cmd)
        
if __name__ == '__main__':

    rospy.init_node('RIDGEBACK', anonymous=True)

    Rid = Ridgeback()
    # Rid.relative_rotate(10)
    Rid.relative_rotate(-10)
    rospy.sleep(2)
    rospy.spin()