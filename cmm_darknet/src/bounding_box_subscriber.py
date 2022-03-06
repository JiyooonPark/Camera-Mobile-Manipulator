#!/usr/bin/env python
from __future__ import division
import roslib
import rospy
from std_msgs.msg import String
from darknet_ros_msgs.msg import *

class BoundingBox:

    def __init__(self):
        self.image_pub = rospy.Publisher("/cmm/darknet_ros",String, queue_size=10)
        self.image_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback)
        self.image_count = rospy.Subscriber("/darknet_ros/found_object",ObjectCount,self.callback_count)
        self.screen = [800, 800]

    def callback(self,msg):
        for box in msg.bounding_boxes:
            # print(box.ymin, box.ymax)

            if box.Class == "person":
                x_mid = (box.xmin+box.xmax)/(self.screen[0]*2)
                y_mid = (box.ymin+box.ymax)/(self.screen[1]*2)
                size = ((box.xmax-box.xmin)*(box.ymax-box.ymin))/(self.screen[0]*self.screen[1])
                pub_msg = "%s/%s/%s" % (x_mid, y_mid, size)
                self.image_pub.publish(pub_msg)

    def callback_count(self,msg):
        # print(msg.count)
        if msg.count == 0:
            pub_msg = "0/0/0"
            self.image_pub.publish(pub_msg)
            


def main():
    rospy.init_node('BoundingBox', anonymous=True)
    try:
        BoundingBox()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    rospy.init_node('BoundingBox', anonymous=True)
    main()
