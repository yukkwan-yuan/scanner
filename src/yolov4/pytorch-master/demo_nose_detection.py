#!/usr/bin/env python3

# -*- coding: utf-8 -*-
'''
@Time          : 20/04/25 15:49
@Author        : huguanghao
@File          : demo.py
@Noice         :
@Modificattion :
    @Author    :
    @Time      :
    @Detail    :
'''

# import sys
# import time
# from PIL import Image, ImageDraw
# from models.tiny_yolo import TinyYoloNet
from tool.utils import *
from tool.torch_utils import *
from tool.darknet2pytorch import Darknet
import argparse
import os
import sys
import math
import rospy
from sensor_msgs.msg import Image as ROSImage
from detection_msgs.msg import Detection2D, BBox2D
from detection_msgs.srv import Detection2DTrig, Detection2DTrigResponse
from cv_bridge import CvBridge, CvBridgeError
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2


"""hyper parameters"""
use_cuda = True
bridge = CvBridge()
# INTEREST_CLASSES = ["green_milk", "black_milk", "oolong_milk"]
#INTEREST_CLASSES = ["Soda", "Coke", "PinkSoda", "Lemonade", "MineralWater"]
INTEREST_CLASSES = ["nose"]
class DetectedImgNode(object):
    def __init__(self, cfgfile, weightfile):
        self.m = Darknet(cfgfile)
        self.m.print_network()
        self.m.load_weights(weightfile)
        print('Loading weights from %s... Done!' % (weightfile))

        if use_cuda:
            self.m.cuda()

        self.pub_msg = rospy.Publisher("~det2d_result", Detection2D, queue_size=1)
        self.sub_img = rospy.Subscriber("/camera1/color/image_raw", ROSImage, self.img_callback, queue_size=1)
        self.detection_srv = rospy.Service("~yolo_detect", Detection2DTrig, self.srv_callback)

    def srv_callback(self, req):
        try:
            cv_image = bridge.imgmsg_to_cv2(req.image, "rgb8")
        except CvBridgeError as e:
            print(e)
            return

        num_classes = self.m.num_classes
        # if num_classes == 20:
        #     namesfile = os.path.join(os.path.dirname(__file__), './data/voc.names')
        # elif num_classes == 80:
        #     namesfile = os.path.join(os.path.dirname(__file__), './data/coco.names')
        # else:
            #namesfile = os.path.join(os.path.dirname(__file__), './data/obj_bottle.names')
        namesfile = os.path.join(os.path.dirname(__file__), './yukkwan/20220103/nose.names')
        class_names = load_class_names(namesfile)

        img_sized = cv2.resize(cv_image, (self.m.width, self.m.height))
        boxes_batch = do_detect(self.m, img_sized, 0.5, 0.2, use_cuda)

        detection_msg = Detection2D()
        detection_msg.header.stamp = rospy.Time.now()
        detection_msg.header.frame_id = req.image.header.frame_id

        # Batch size != 1
        if len(boxes_batch) != 1:
            print("Batch size != 1, cannot handle it")
            exit(-1)
        boxes = boxes_batch[0]
        
        for index, box in enumerate(boxes):
            bbox_msg = BBox2D()
            bbox_msg.center.x = math.floor(box[0] * req.image.width)
            bbox_msg.center.y = math.floor(box[1] * req.image.height)
            bbox_msg.size_x = math.floor(box[2] * req.image.width)
            bbox_msg.size_y = math.floor(box[3] * req.image.height)
            bbox_msg.id = box[6]
            bbox_msg.score = box[5]
            bbox_msg.class_name = class_names[bbox_msg.id]
            detection_msg.boxes.append(bbox_msg)
        
        result_img = plot_boxes_cv2(cv_image, boxes, savename=None, class_names=class_names, interest_classes=INTEREST_CLASSES)
        # result_img = plot_boxes_cv2(cv_image, boxes, savename=None, class_names=class_names)
        detection_msg.result_image = bridge.cv2_to_imgmsg(result_img, "rgb8")

        #print('return {} detection results'.format(len(boxes)))
        return Detection2DTrigResponse(result=detection_msg)

    def img_callback(self, msg):
        try:
            imgfile = bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            print(e)
            return

        num_classes = self.m.num_classes
        # if num_classes == 20:
        #     namesfile = os.path.join(os.path.dirname(__file__), './data/voc.names')
        # elif num_classes == 80:
        #     namesfile = os.path.join(os.path.dirname(__file__), './data/coco.names')
        # else:
            #namesfile = os.path.join(os.path.dirname(__file__), './data/obj_bottle.names')
        namesfile = os.path.join(os.path.dirname(__file__), './yukkwan/20220103/nose.names')
        class_names = load_class_names(namesfile)

        sized = cv2.resize(imgfile, (self.m.width, self.m.height))

        for i in range(2):
            start = time.time()
            boxes = do_detect(self.m, sized, 0.4, 0.3, use_cuda)
            finish = time.time()
            # if i == 1:
            #     print('\nPredicted in %f seconds.\n' % (finish - start))
            #     print('=====================================================')

        detection_msg = Detection2D()
        detection_msg.header.stamp = rospy.Time.now()
        detection_msg.header.frame_id = msg.header.frame_id
        
        for index, box in enumerate(boxes[0]):
            bbox_msg = BBox2D()
            bbox_msg.center.x = math.floor(box[0] * msg.width)
            bbox_msg.center.y = math.floor(box[1] * msg.height)
            bbox_msg.size_x = math.floor(box[2] * msg.width)
            bbox_msg.size_y = math.floor(box[3] * msg.height)
            bbox_msg.id = box[6]
            bbox_msg.score = box[5]
            bbox_msg.class_name = class_names[bbox_msg.id]
            detection_msg.boxes.append(bbox_msg)
            x = math.floor(box[0] * msg.width)
            
        result_img = plot_boxes_cv2(sized, boxes[0], savename=None, class_names=class_names, interest_classes=INTEREST_CLASSES)
        # result_img = plot_boxes_cv2(imgfile, boxes[0], savename=None, class_names=class_names)
        detection_msg.result_image = bridge.cv2_to_imgmsg(result_img, encoding="rgb8")
        # self.pub_img.publish(bridge.cv2_to_imgmsg(result_img, encoding="rgb8"))
        self.pub_msg.publish(detection_msg)

if __name__ == '__main__':
    #rospy.loginfo("Use yolov4 model")

    # weightfile = os.path.join(os.path.dirname(__file__), "./weights/yolov4-tiny.weights")
    # cfgfile = os.path.join(os.path.dirname(__file__), "./cfg/yolov4-tiny.cfg")
    # weightfile = os.path.join(os.path.dirname(__file__), "./weights/milktea/yolov4-tiny-obj_6000.weights")
    # cfgfile = os.path.join(os.path.dirname(__file__), "./cfg/yolov4-tiny-obj.cfg")
    # weightfile = os.path.join(os.path.dirname(__file__), "./weights/yolov4-tiny-nose_final.weights")
    # cfgfile = os.path.join(os.path.dirname(__file__), "./cfg/yolov4-tiny-nose.cfg")
    weightfile = os.path.join(os.path.dirname(__file__), "./yukkwan/20220103/yolov4-tiny-nose_3000.weights")
    cfgfile = os.path.join(os.path.dirname(__file__), "./yukkwan/20220103/yolov4-tiny-nose.cfg")

    rospy.init_node('DetectedImgNode', anonymous=False)       
    node = DetectedImgNode(cfgfile, weightfile)
    rospy.spin()
