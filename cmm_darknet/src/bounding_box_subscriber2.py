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
        self.screen = [1125, 800]
        self.linear_x = []
        self.linear_y = []
        self.x_mid_list = []
        

    def callback(self,msg):
        person_list = []
        for box in msg.bounding_boxes:
            # print(box.ymin, box.ymax)

            if box.Class == "person":
                person_list.append(box)
                
        if len(person_list) > 2:
            person = max(person_list, key=lambda item: item.probability)
        elif len(person_list) == 1:
            person = person_list[0]
        else:
            return
        # if len(person_list) != 0:
        x_mid = (person.xmin+person.xmax)/(self.screen[0]*2)
        y_mid = (person.ymin+person.ymax)/(self.screen[1]*2)
        size = (person.xmax-person.xmin)/(self.screen[0])
        pub_msg = "%s/%s" % (x_mid, size)
        print(pub_msg)
        self.linear_x.append(x_mid)
        self.linear_y.append(y_mid)
        self.x_mid_list.append(x_mid)

        # else:
        #     pub_msg = "0/0"
            
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
