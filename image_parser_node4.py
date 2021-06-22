#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import BEVTransform


class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

        self.source_prop = np.float32([[0.05, 0.60],
                                       [0.5 - 0.25, 0.52],
                                       [0.5 + 0.25, 0.52],
                                       [1 - 0.05, 0.60]
                                      ])
        self.img_lane = None
    def callback(self,msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        
        self.img_hsv = cv2.cvtColor(self.img_bgr,cv2.COLOR_BGR2HSV)

        self.lower_wlane = np.array([0,0,210])
        self.upper_wlane = np.array([50,50,250])
        
        self.lower_ylane = np.array([0,100,190])
        self.upper_ylane = np.array([40,175,255])

        self.img_wlane = cv2.inRange(self.img_hsv, self.lower_wlane, self.upper_wlane)
        self.img_ylane = cv2.inRange(self.img_hsv, self.lower_ylane, self.upper_ylane)

        # self.img_lane = cv2.bitwise_or(self.img_wlane, self.img_ylane)
        img_lane = cv2.bitwise_or(self.img_wlane, self.img_ylane)
        img_lane[:int(self.img_bgr.shape[0]/2) , :] =0
        self.img_lane = img_lane



if __name__ == '__main__':
    # rospy.init_node('image_parser', anonymous=True)

    # image_parser = IMGParser()

    # rospy.spin()
    rp = rospkg.RosPack()
    currentPath = rp.get_path("sensor_parse_example")

    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('IMGParser', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam, xb=6, zb=5)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        if image_parser.img_lane is not None:

            img_warp = bev_op.warp_bev_img(image_parser.img_lane)
            cv2.imshow("Image window", img_warp)
            cv2.waitKey(1)

            rate.sleep()