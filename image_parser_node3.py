#!/usr/bin/env python

from time import clock
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import warp_image

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

        self.source_prop = np.float32([[0.05, 0.60],
                                       [0.5 - 0.25, 0.52],
                                       [0.5 + 0.25, 0.52],
                                       [1 - 0.05, 0.60]
                                      ])

    def callback(self,msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        
        self.img_hsv = cv2.cvtColor(self.img_bgr,cv2.COLOR_BGR2HSV)

        self.lower_wlane = np.array([10,15,210])
        self.upper_wlane = np.array([50,50,250])
        
        self.lower_wlane = np.array([0,100,190])
        self.upper_wlane = np.array([40,175,255])

        self.img_wlane = cv2.inRange(self.img_hsv, self.lower_wlane, self.upper_wlane)
        self.img_ylane = cv2.inRange(self.img_hsv, self.lower_wlane, self.upper_wlane)

        self.img_lane = cv2.bitwise_or(self.img_wlane, self.img_ylane)

        self.img_warp = warp_image(self.img_lane, self.source_prop)

        img_concat = np.concatenate([self.img_lane, self.img_warp], axis = 1)

        cv2.imshow('Image window', img_concat)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()