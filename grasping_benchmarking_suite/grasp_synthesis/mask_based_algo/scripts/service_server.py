#!/usr/bin/env python

import rospy

from mask_based_algo_module.grasp_mask import GraspMask
from mask_based_algo.srv import Grasp2DPrediction, Grasp2DPredictionResponse

import cv_bridge
bridge = cv_bridge.CvBridge()

class GraspService:
    def __init__(self):
        rospy.Service('~predict', Grasp2DPrediction, self.service_cb)

    def service_cb(self, data):
        depth = bridge.imgmsg_to_cv2(data.depth_image)
        
        grasp_mask = GraspMask(min(depth.shape))
        x, y, angle, width = grasp_mask.get_grasp(depth)
 
        response = Grasp2DPredictionResponse()
        g = response.best_grasp

        # Scale detection for correct 3D transformation
        g.px = x
        g.py = y
        g.angle = angle + 1.57
        g.width = width
        g.quality = 1

        rospy.loginfo("Grasp in Image frame: %s, %s, %s (%s)", g.px, g.py, g.angle, (depth.shape))

        return response
    

if __name__ == '__main__':
    rospy.init_node('mask_based_grasp_service')
        
    grasp_service = GraspService()
    # grasp_mask = GraspMask()
    
    rospy.spin()