#!/usr/bin/env python

import rospy
import numpy as np

# Define your algorithm to take depth/rgb images in the predict function and call that function here
#from template_algo.template_algo_torch import predict

from template_algo.srv import Grasp2DPrediction, Grasp2DPredictionResponse

import cv_bridge
bridge = cv_bridge.CvBridge()

class GraspService:
    def __init__(self):
        rospy.Service('~predict', Grasp2DPrediction, self.service_cb)

    def service_cb(self, data):
        depth = bridge.imgmsg_to_cv2(data.depth_image)
        rgb = bridge.imgmsg_to_cv2(data.rgb_image)
        
        #points, angle, width_img, _ = predict(depth, rgb) Implement your own predict function

  
        response = Grasp2DPredictionResponse()
        g = response.best_grasp

        # Update the best_grasp in the response
        #x, y = np.unravel_index(np.argmax(points), points.shape)
        #ang = angle[x][y]
        #g.px = int(x*depth.shape[0]/300)
        #g.py = int(y*depth.shape[0]/300 + (depth.shape[1] - depth.shape[0])/2)
        #g.angle = ang
        #g.width = int(width_img[x][y]*depth.shape[0]/300)
        #g.quality = points[x][y]

        #rospy.loginfo("Grasp in Image frame: %s, %s, %s (%s)", g.px, g.py, g.angle, (depth.shape))

        return response

if __name__ == '__main__':
    rospy.init_node('template_algo_grasp_service')
    GraspService()
    rospy.spin()


