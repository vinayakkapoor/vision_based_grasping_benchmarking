#!/usr/bin/env python3
import sys
from pathlib import Path

# Set root directory
ROOT = Path("../")
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np
from grasp import run_detector
from grasp_interfaces.srv import Grasp2DPrediction
import cv_bridge
from sensor_msgs.msg import Image

bridge = cv_bridge.CvBridge()

class GraspService(Node):
    def __init__(self):
        super().__init__('resnet_deep_grasp_service')
        self.srv = self.create_service(
            Grasp2DPrediction, 'predict', self.service_cb, 
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def service_cb(self, request, response):
        depth = bridge.imgmsg_to_cv2(request.depth_image)
        rgb = bridge.imgmsg_to_cv2(request.rgb_image)
        
        rgd = rgb.copy()
        rgd[:, :, 2] = depth

        center, angle = run_detector(rgd)

        response.best_grasp.px = int(center[1])
        response.best_grasp.py = int(center[0])
        response.best_grasp.angle = angle + 1.57
        response.best_grasp.width = 0
        response.best_grasp.quality = 0
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

