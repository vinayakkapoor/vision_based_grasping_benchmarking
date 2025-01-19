#!/usr/bin/env python3

import rospy
import numpy as np
import math
import cv2
from sensor_msgs.msg import Image

from ggcnn.ggcnn_torch import predict, process_depth_image
from ggcnn.srv import GraspPrediction, GraspPredictionResponse  

import cv_bridge
bridge = cv_bridge.CvBridge()

class GraspService:
    def __init__(self, sim_mode=False, crop=True):
        self.sim_mode = sim_mode
        self.crop = crop
        # Full image: [0, 0, 720, 1280]

        # self.crop_size = [110, 197, 720, 1083]
        # self.crop_size = [110, 295, 720, 1181] 
        self.crop_size = [170, 355, 660, 1121] 

        if self.sim_mode:
            rospy.Subscriber("", Image, self.rgb_cb)
            rospy.Subscriber("", Image, self.depth_cb)
        else:
            rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_cb)
            # rospy.Subscriber("/camera/aligned_depth_to_color/depth_completed", Image, self.depth_cb)
            rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_cb)

        rospy.Service('debug', GraspPrediction, self.service_cb)

        self.rgb_cropped_pub = rospy.Publisher("cropped_rgb", Image, queue_size=10)
        self.depth_cropped_pub = rospy.Publisher("cropped_depth", Image, queue_size=10) 

        self.curr_depth_img = None
        self.curr_rgb_img = None

    def depth_cb(self, msg):
        img = bridge.imgmsg_to_cv2(msg)
        if self.crop:
            self.curr_depth_img = img[self.crop_size[0]:self.crop_size[2], self.crop_size[1]:self.crop_size[3]]
            self.depth_cropped_pub.publish(bridge.cv2_to_imgmsg(self.curr_depth_img))
        else:
            self.curr_depth_img = img 
        self.received = True

    def rgb_cb(self, msg):
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.crop:
            self.curr_rgb_img = img[self.crop_size[0]:self.crop_size[2], self.crop_size[1]:self.crop_size[3], :]
            self.rgb_cropped_pub.publish(bridge.cv2_to_imgmsg(self.curr_rgb_img, encoding='bgr8'))
        else:
            self.curr_rgb_img = img

    def service_cb(self, data):
        depth = self.curr_depth_img
        rgb = self.curr_rgb_img

        #####################################################################
        # Insert your algorithm specific code here

        depth_crop, depth_nan_mask = process_depth_image(depth, depth.shape[0], 300, return_mask=True, crop_y_offset=0)
        points, angle, width_img, _ = predict(depth_crop, process_depth=False, depth_nan_mask=depth_nan_mask, filters=(2.0, 2.0, 2.0))

        x, y = np.unravel_index(np.argmax(points), points.shape)
        ang = angle[x][y]

        response = GraspPredictionResponse()
        g = response.best_grasp

        # Scale detection for correct 3D transformation
        g.pose.position.x = int(x*depth.shape[0]/300) 
        g.pose.position.y = int(y*depth.shape[0]/300 + (depth.shape[1] - depth.shape[0])/2)
        g.pose.orientation.z = ang
        g.width = int(width_img[x][y]*depth.shape[0]/300)

        print(f"Grasp Detected - x: {x} y: {y}")
        print(f"Accounting for crop - x: {g.pose.position.x}. y: {g.pose.position.y}")

        bb = self.draw_angled_rect(rgb, g.pose.position.y, g.pose.position.x, g.pose.orientation.z)

        cv2.imshow('Grasp Detection', bb)
        norm_image = cv2.normalize(depth, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

        points = cv2.cvtColor(points,cv2.COLOR_GRAY2RGB)
        angle = cv2.cvtColor(angle,cv2.COLOR_GRAY2RGB)
        points = cv2.circle(points, (y, x), 3, (0, 0, 255), -1)
        angle = cv2.circle(angle, (y, x), 3, (0, 0, 255), -1)

        cv2.imshow('Depth', norm_image)
        cv2.imshow('Points', points)
        cv2.imshow('Angle', angle)
        cv2.waitKey(0)

        ########################################################################

        return response

    def draw_angled_rect(self, image, x, y, angle, width = 220, height = 100):
        b = math.cos(angle) * 0.5
        a = math.sin(angle) * 0.5

        display_image = image.copy()

        pt0 = (int(x - a * height - b * width), int(y + b * height - a * width))
        pt1 = (int(x + a * height - b * width), int(y - b * height - a * width))
        pt2 = (int(2 * x - pt0[0]), int(2 * y - pt0[1]))
        pt3 = (int(2 * x - pt1[0]), int(2 * y - pt1[1]))

        cv2.line(display_image, pt0, pt1, (255, 0, 0), 5)
        cv2.line(display_image, pt1, pt2, (0, 0, 0), 5)
        cv2.line(display_image, pt2, pt3, (255, 0, 0), 5)
        cv2.line(display_image, pt3, pt0, (0, 0, 0), 5)
        cv2.circle(display_image, ((pt0[0] + pt2[0])//2, (pt0[1] + pt2[1])//2), 3, (0, 0, 0), -1)
        return display_image

if __name__ == '__main__':
    rospy.init_node('grasp_service')
    GraspService()
    rospy.spin()
