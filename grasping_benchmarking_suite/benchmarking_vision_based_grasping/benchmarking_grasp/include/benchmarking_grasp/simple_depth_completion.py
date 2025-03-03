#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial import KDTree
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

class DepthCompletion(Node):
    def __init__(self):
        super().__init__('simple_depth_completion')
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter("align_depth", False)
        self.declare_parameter("depth_image", "")
        self.declare_parameter("depth_wo_align_image", "")
        self.declare_parameter("depth_complete_image", "")
        self.declare_parameter("depth_complete_image_norm", "")
        
        align_depth = self.get_parameter("align_depth").value
        self.normalize = False  # Maintain original code behavior

        # Get topic parameters
        if align_depth:
            depth_image_topic = self.get_parameter("depth_image").value
        else:
            depth_image_topic = self.get_parameter("depth_wo_align_image").value

        depth_complete_image_topic = self.get_parameter("depth_complete_image").value
        depth_complete_image_norm_topic = self.get_parameter("depth_complete_image_norm").value

        # Create subscriber with QoS profile depth=1
        self.subscription = self.create_subscription(
            Image,
            depth_image_topic,
            self._depth_img_cb,
            qos_profile=1
        )

        # Create publishers with QoS profile depth=1
        self.depth_complete_pub = self.create_publisher(Image, depth_complete_image_topic, 1)
        
        if self.normalize:
            self.depth_complete_norm_pub = self.create_publisher(Image, depth_complete_image_norm_topic, 1)

        self.get_logger().info("Depth completion node initialized")

    def complete_depth_kdtree(self, image):
        '''Replaces zero pixels with nearest nonzero pixel using KDTree'''
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
        try:
            img = self.bridge.imgmsg_to_cv2(msg)
            img = self.complete_depth_kdtree(img.copy())
            self.depth_complete_pub.publish(self.bridge.cv2_to_imgmsg(img))

            if self.normalize:
                normalized_depth_image = (img - np.min(img)) * 255 / (np.max(img) - np.min(img))
                normalized_depth_image = np.uint8(normalized_depth_image)
                self.depth_complete_norm_pub.publish(self.bridge.cv2_to_imgmsg(normalized_depth_image))
                
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthCompletion()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
