#!/usr/bin/env python

import sys
ROOT = '../'  # root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

import rospy
import numpy as np
import math
import cv2

from grasp import run_detector
from ggcnn.srv import Grasp2DPrediction, Grasp2DPredictionResponse, Grasp2DPredictionRequest

import cv_bridge
bridge = cv_bridge.CvBridge()

class GraspService:
    def __init__(self):
        rospy.Service('~predict', Grasp2DPrediction, self.service_cb)

    def service_cb(self, data):
        depth = bridge.imgmsg_to_cv2(data.depth_image)
        rgb = bridge.imgmsg_to_cv2(data.rgb_image)
        
        rgd = rgb.copy()
        rgd[:, :, 2] = depth

        center, angle = run_detector(rgd)

        x = center[1]
        y = center[0]

        response = Grasp2DPredictionResponse()
        g = response.best_grasp
        g.px = int(x)
        g.py = int(y)
        g.angle = angle + 1.57
        g.width = 0
        g.quality = 0

        print(g.px, g.py, depth.shape)
        print(x, y, depth.shape)

        return response

if __name__ == '__main__':
    rospy.init_node('resnet_deep_grasp_service')
    GraspService()
    rospy.spin()


