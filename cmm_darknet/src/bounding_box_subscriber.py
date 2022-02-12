#!/usr/bin/env python
from __future__ import division
import roslib
import rospy
from std_msgs.msg import String
from darknet_ros_msgs.msg import *

class BoundingBox:

    def __init__(self):
        self.image_pub = rospy.Publisher("bounding_box",String, queue_size=10)

        self.image_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback)

    def callback(self,msg):
        # box = msg.bounding_boxes[0]
        # print(1)
        # print(msg.bounding_boxes)
        for box in msg.bounding_boxes:
            print(box.Class)
            if box.Class == "person":
                # print(box.xmin)
                x_mid = (box.xmin+box.xmax)/1650
                y_mid = (box.ymin+box.ymax)/1650
                # print(box.xmin+box.xmax)
                # print(x_mid)
                pub_msg = "x: %s y: %s" % (x_mid, y_mid)

            else:
                pub_msg = "person not detected"

            self.image_pub.publish(pub_msg)

def main(args):
    rospy.init_node('BoundingBox', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    rospy.init_node('BoundingBox', anonymous=True)
    BoundingBox()
    rospy.spin()

