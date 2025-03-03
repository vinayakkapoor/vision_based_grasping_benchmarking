#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
import numpy as np 
import cv_bridge as cvb

bridge = cvb.CvBridge()
import os
cwd = os.getcwd()
print(cwd)


#import ggcnn model from ggcnn_ros2/src/ggcnn/ 
from ggcnn_module.ggcnn_torch import predict, process_depth_image

from grasp_interfaces.srv import Grasp2DPrediction


class GraspService(Node):
    def __init__(self):
        super().__init__('grasp_service')
        # in ros2 no need to add ~predict to append to node name
        self.srv = self.create_service(Grasp2DPrediction,'/ggcnn_grasp_service/predict',self.service_cb) 

    def service_cb(self,request,response):
        depth = bridge.imgmsg_to_cv2(request.depth_image)
        depth_crop, depth_nan_mask = process_depth_image(depth, depth.shape[0], 300, return_mask=True, crop_y_offset=0)
        points, angle, width_img, _ = predict(depth_crop, process_depth=False, depth_nan_mask=depth_nan_mask, filters=(2.0, 2.0, 2.0))

        x, y = np.unravel_index(np.argmax(points), points.shape)
        ang = angle[x][y]

        g = response.best_grasp
        
        # Scale detection for correct 3D transformation
        g.px = int(x * depth.shape[0] / 300)
        g.py = int(y * depth.shape[0] / 300 + (depth.shape[1] - depth.shape[0]) / 2)
        g.angle = float(ang)
        g.width = float(width_img[x][y] * depth.shape[0] / 300)
        g.quality = float(points[x][y])

        return response



def main():
    "main function"
    rclpy.init()
    grasp_service = GraspService()
    rclpy.spin(grasp_service)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
