#!/usr/bin/env python
import rospy
from  sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import cv2

import os
import sys
import time
import numpy as np

rgb = None
depth = None
lock = False
bridge = CvBridge()
K_cam = None

def receiveRGBD(rgb_, depth_):
    print("RGBD start")
    global lock, rgb, depth
    if not lock:
        rgb = bridge.imgmsg_to_cv2(rgb_,"bgr8")
        depth_.encoding = "mono16"
        depth = bridge.imgmsg_to_cv2(depth_,"mono16")
        lock = True
        # rgb = bridge.imgmsg_to_cv2(rgb_.data, "bgr8")
        # depth = bridge.imgmsg_to_cv2(depth_.data, "mono16")


def match():
    global lock, rgb, depth
    if lock:
        print(depth[100,100])

        # print depth[10,10]
        lock = False

def getK(info, subscriber):
    #do processing here to get K_cam
    global K_cam
    K_cam = np.zeros(shape=(3,3))
    # for i in range(9):
        # K_cam[i % 3, i - (i % 3) * 3] = info.K[i]
    
    # subscriber.unregister()

if __name__ == '__main__':
    rospy.init_node('linemod_detection')
    # get K from cam_info topic
    sub_once = None
    sub_once = rospy.Subscriber('camera/color/camera_info', CameraInfo, getK, [sub_once])
    # while K_cam == None:
    #     pass
    # rospy.Subscriber('/camera/color/image_raw', Image, test, queue_size=2)
    rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image, queue_size=2)
    depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image, queue_size=2)
    queue_size = 2
    slop_seconds = 0.025
    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size, slop_seconds)
    ts.registerCallback(receiveRGBD)

    while not rospy.is_shutdown():
        match()

    rospy.spin()