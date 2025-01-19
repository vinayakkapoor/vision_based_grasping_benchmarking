#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.spatial import KDTree as KDTree
import cv2
import cv_bridge

from sensor_msgs.msg import Image

class DepthCompletion:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        align_depth = rospy.get_param("align_depth") 
        self.normalize = False

        if align_depth:
            depth_image_topic = rospy.get_param("depth_image")
        else:
            depth_image_topic = rospy.get_param("depth_wo_align_image")

        depth_complete_image_topic = rospy.get_param("depth_complete_image")
        depth_complete_image_norm_topic = rospy.get_param("depth_complete_image_norm")

        rospy.Subscriber(depth_image_topic, Image, self._depth_img_cb, queue_size=1)
        self.depth_complete_pub = rospy.Publisher(depth_complete_image_topic, Image, queue_size=1)
        
        if self.normalize:
            self.depth_complete_norm_pub = rospy.Publisher(depth_complete_image_norm_topic, Image, queue_size=1)

    def complete_depth_kdtree(self, image):
        '''
        replaces nonzero pixels with the nearest nonzero pixel
        '''
        
        thresh = 0.1
        valid_pixels = np.argwhere(image > thresh)
        invalid_pixels = np.argwhere(image <= 0)
        
        if len(valid_pixels) == 0:
            return image

        kdtree = KDTree(valid_pixels)
        _, pre_indices = kdtree.query(invalid_pixels, k=1)
        
        indices = valid_pixels[pre_indices]
        image[invalid_pixels[:, 0], invalid_pixels[:, 1]] = image[indices[:, 0], indices[:, 1]]
        
        return image

    def _depth_img_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        img = self.complete_depth_kdtree(img.copy())
        self.depth_complete_pub.publish(self.bridge.cv2_to_imgmsg(img))

        if self.normalize:
            normalized_depth_image = (img - np.min(img)) * 255 / (np.max(img) - np.min(img))
            normalized_depth_image = np.uint8(normalized_depth_image)
            self.depth_complete_norm_pub.publish(self.bridge.cv2_to_imgmsg(normalized_depth_image))

if __name__ == "__main__":
    rospy.init_node("simple_depth_completion", log_level=rospy.INFO)
    depth_completion = DepthCompletion()

    rospy.spin()
    