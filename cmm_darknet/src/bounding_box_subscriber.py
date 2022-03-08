#!/usr/bin/env python
from __future__ import division
import roslib
import rospy
from std_msgs.msg import String
from darknet_ros_msgs.msg import *

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

class BoundingBox:

    def __init__(self):
        self.image_pub = rospy.Publisher("/cmm/darknet_ros",String, queue_size=10)
        self.image_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback)
        self.image_count = rospy.Subscriber("/darknet_ros/found_object",ObjectCount,self.callback_count)
        self.screen = [1125, 800]
        self.linear_x = []
        self.linear_y = []
        self.x_mid_list = []

    def callback(self,msg):
        for box in msg.bounding_boxes:
            if box.Class == "person" and box.probability > 0.65:
                x_mid = (box.xmin+box.xmax)/(self.screen[0]*2)
                y_mid = (box.ymin+box.ymax)/(self.screen[1]*2)
                size = ((box.xmax-box.xmin)*(box.ymax-box.ymin))/(self.screen[0]*self.screen[1])
                pub_msg = "%s/%s/%s" % (x_mid, y_mid, size)
                self.image_pub.publish(pub_msg)
                self.linear_x.append(x_mid)
                self.linear_y.append(y_mid)
                self.x_mid_list.append(x_mid)


    def callback_count(self,msg):
        # print(msg.count)
        if msg.count == 0:
            pub_msg = "0/0/0"
            self.image_pub.publish(pub_msg)
    
    def animate(self, i):
        ax.clear()
        ax.set_title('Person Pose')
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)

        ax.plot(self.linear_x, self.linear_y, color='r')

    def start(self):
        ani = animation.FuncAnimation(fig, self.animate, interval=10)
        plt.show()


def main():
    rospy.init_node('BoundingBox', anonymous=True)
    try:
        B = BoundingBox()

        B.start()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    rospy.init_node('BoundingBox', anonymous=True)
    fig = plt.figure()
    ax = fig.gca()
    main()
