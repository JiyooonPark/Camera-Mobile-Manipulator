#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import math, time
import numpy as np
from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose, Twist
from std_msgs.msg import *

class Ridgeback:

    def __init__(self):

        self.publisher_pose = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback/pose', Pose, queue_size=10)
        self.publisher_state = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback/state', String, queue_size=10)
        self.publisher_traj = rospy.Publisher('/iiwa_ridgeback_communicaiton/trajectory', PoseArray, queue_size=100)
        self.publisher_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subscriber_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odom)
        self.subscriber_iiwa = rospy.Subscriber("/iiwa_ridgeback_communicaiton/iiwa", String, self.callback_iiwa)
        self.state = 0
        self.linear_speed = 0.1
        self.reached = False
        self.i = 0

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

    def callback_iiwa(self, msg):
        self.iiwa_state = msg

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

        self.move_relative(float(r_goal[0][0]), float(r_goal[1][0]), duration=10)

    def move_relative(self, x, y, duration=5):
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
        print(f'Executing ridgeback drawing #{self.state}')
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

    def follow_trajectory(self, path, angles):

        for i in range(len(angles)):
            while self.iiwa_state != '0':
                rospy.sleep(1)
            print(f'IIWA done executing #{self.iiwa_state}')

            print(f'i: {i}, x: {path[i][0]}, y: {path[i][1]}')

            self.fixed_rotate(0)

            result = self.fixed_goal(path[i][0], path[i][1]+0.5)

            if result:
                rospy.loginfo("Goal execution done!")
                self.publish_state(self.state)
                self.state += 1
                self.publish_pose()

            if angles[i][0] == 'l':
                self.fixed_rotate(abs(float(angles[i][2:]))+180)
            else:
                self.fixed_rotate(360 - abs(float(angles[i][2:])))

            self.fixed_rotate(0)
            rospy.sleep(0.5)

    def publish_state(self, number):
        self.publisher.publish(number)

    def publish_pose(self):
        
        P = Pose()
        P.position.x = self.position_x
        P.position.y = self.position_y
        P.orientation.x = self.yaw

        self.publish_pose(P)

    def fixed_rotate(self, target_angle):

        kp=0.5
        command =Twist()

        if 0<=target_angle<=180:
            real_target = -target_angle
        else:
            real_target = 360-target_angle

        while True:
            target_rad = real_target*math.pi/180
            command.angular.z = kp * (target_rad-self.yaw)
            self.publisher_cmd_vel.publish(command)

            if abs(self.yaw*180/math.pi-real_target)<1:
                break

    def rotate_relative( self,duration=3):
        print("rotate")
        publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        cmd = Twist()
        cmd.angular.z = 0.05

        rospy.sleep(1)

        seconds = time.time()
        while time.time() - seconds < duration:
            publisher.publish(cmd)

    def publish_trajectory(self):
        
        self.path_angle, self.iiwa_range_list, self.path_x, self.path_y = run_algorithm()

        message= PoseArray()
        pose_list = []
        
        for i in range(len(self.path_x)):
            p = Point()
            q = Quaternion()
            pose = Pose()

            p.x = self.path_x[i]
            p.y = self.path_y[i]

            if self.path_angle[i][0] == 'l':
                q.x = abs(float(self.path_angle[i][2:]))+180
            else:
                q.x = 360 - abs(float(self.path_angle[i][2:]))

            pose.position = p
            pose.orientation = q

            pose_list.append(pose)

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        message.header = h

        message.poses = pose_list

        rate = rospy.Rate(10) 

        for i in range(10):
            self.publisher_traj.publish(message)
            rate.sleep()

if __name__ == '__main__':

    rospy.init_node('RIDGEBACK', anonymous=True)

    Rid = Ridgeback()
    rospy.sleep(2)
    # Rid.publish_trajectory()
    # Rid.rotate_relative()
    Rid.move_relative(0.1, 0)
    # Rid.follow_trajectory(list(zip(Rid.path_x, Rid.path_y)), Rid.path_angle)
