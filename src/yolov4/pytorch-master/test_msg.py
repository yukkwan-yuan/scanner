#!/usr/bin/env python3

import rospy
import os
import sys
import numpy as np
from detection_msgs.msg import Detection2D, BBox2D
ros_py27_path = '/opt/ros/' + os.getenv("ROS_DISTRO") + '/lib/python2.7/dist-packages'
if ros_py27_path in sys.path:
    sys.path.remove(ros_py27_path)
import cv2
from cv_bridge import CvBridge
bridge = CvBridge()

def callback(msg):
    if len(msg.boxes) != 0:
        img = bridge.imgmsg_to_cv2(msg.result_image, "rgb8")
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow("test", img)
        cv2.waitKey(1)

        for i in range(len(msg.boxes)):
            print("Target Name = ", msg.boxes[i].class_name)
            print("x = ", msg.boxes[i].center.x)
            print("y = ", msg.boxes[i].center.y)
    else:
        print("No target detected !!!!!\n")

    print("\n")

if __name__ == '__main__':
    sub_img = rospy.Subscriber("/DetectedImgNode/det2D_result", Detection2D, callback, queue_size=1)
    rospy.init_node('test_node', anonymous=False)
    rospy.spin()