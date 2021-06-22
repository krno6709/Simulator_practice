#!/usr/bin/env python

from time import clock
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self,msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        
        self.img_hsv = cv2.cvtColor(self.img_bgr,cv2.COLOR_BGR2HSV)
        self.img_concat = np.concatenate([self.img_bgr, self.img_hsv], axis = 1)

        # cv2.imshow("Image window", self.img_concat)
        # cv2.waitKey(1)
        cv2.namedWindow('mouseRGB')
        cv2.imshow('mouseRGB', self.img_concat)
        cv2.setMouseCallback('mouseRGB', self.mouseRGB)

        # cv2.imshow("Image window", self.img_bgr)
        cv2.waitKey(1)

    def mouseRGB(self, event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            colorsB = self.img_concat[y,x,0]
            colorsG = self.img_concat[y,x,1]
            colorsR = self.img_concat[y,x,2]
            colors = self.img_concat[y,x]
            print("Red: ",colorsR)
            print("Green: ",colorsG)
            print("Blue: ",colorsB)
            print("BRG Format: ",colors)
            print("Coordinates of pixel X: ",x,"y: ",y)


if __name__ == '__main__':
    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()