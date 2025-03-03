#!/usr/bin/env python3

import sys
ROOT = '../'  # root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

import rclpy
from rclpy.node import Node
import numpy as np

from grasp import run_detector
from grasp_interfaces.srv import Grasp2DPrediction
import cv_bridge

class GraspService(Node):
    def __init__(self):
        super().__init__('resnet_deep_grasp_service')
        # Create a service with the name 'predict'
        self.srv = self.create_service(Grasp2DPrediction, 'resnet_deep_grasp_service/predict', self.service_cb)
        self.bridge = cv_bridge.CvBridge()

    def service_cb(self, request, response):
        # Convert ROS image messages to OpenCV images
        depth = self.bridge.imgmsg_to_cv2(request.depth_image)
        rgb = self.bridge.imgmsg_to_cv2(request.rgb_image, desired_encoding='rgb8')
        
        # Create an RGBD image by replacing the red channel with depth data
        rgd = rgb.copy()
        rgd[:, :, 2] = depth

        center, angle = run_detector(rgd)

        x = center[1]
        y = center[0]

        # Fill the response message
        response.best_grasp.px = int(x)
        response.best_grasp.py = int(y)
        response.best_grasp.angle = angle + 1.57
        response.best_grasp.width = 0
        response.best_grasp.quality = 0

        return response

def main(args=None):
    rclpy.init(args=args)
    grasp_service = GraspService()
    rclpy.spin(grasp_service)
    grasp_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
