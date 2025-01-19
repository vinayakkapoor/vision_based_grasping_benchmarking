#!/usr/bin/env python

import rospy
import numpy as np

# To use Tensorflow implementation
# from ggcnn.ggcnn import predict, process_depth_image

# To use Pytorch implementation
from ggcnn.ggcnn_torch import predict, process_depth_image

from ggcnn.srv import Grasp2DPrediction, Grasp2DPredictionResponse

import cv_bridge
bridge = cv_bridge.CvBridge()

class GraspService:
    def __init__(self):
        rospy.Service('~predict', Grasp2DPrediction, self.service_cb)

    def service_cb(self, data):
        depth = bridge.imgmsg_to_cv2(data.depth_image)
        
        depth_crop, depth_nan_mask = process_depth_image(depth, depth.shape[0], 300, return_mask=True, crop_y_offset=0)
        points, angle, width_img, _ = predict(depth_crop, process_depth=False, depth_nan_mask=depth_nan_mask, filters=(2.0, 2.0, 2.0))

        x, y = np.unravel_index(np.argmax(points), points.shape)
        ang = angle[x][y]

        response = Grasp2DPredictionResponse()
        g = response.best_grasp

        # Scale detection for correct 3D transformation
        g.px = int(x*depth.shape[0]/300)
        g.py = int(y*depth.shape[0]/300 + (depth.shape[1] - depth.shape[0])/2)
        g.angle = ang
        g.width = int(width_img[x][y]*depth.shape[0]/300)
        g.quality = points[x][y]

        rospy.logerr("Grasp in Image frame: %s, %s, %s (%s)", g.px, g.py, g.angle, (depth.shape))

        return response

if __name__ == '__main__':
    rospy.init_node('ggcnn_grasp_service')
    GraspService()
    rospy.spin()


