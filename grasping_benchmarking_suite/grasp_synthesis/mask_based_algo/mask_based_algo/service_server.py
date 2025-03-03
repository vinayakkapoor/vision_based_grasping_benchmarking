#!/usr/bin/env python3


import rclpy 
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from mask_based_algo_module.grasp_mask import GraspMask
from grasp_interfaces.srv import Grasp2DPrediction

import cv_bridge
bridge = cv_bridge.CvBridge()

class GraspService(Node):
    def __init__(self):
        super().__init__('mask_based_grasp_service')
        self.srv = self.create_service(Grasp2DPrediction,'/mask_based_grasp_service/predict',self.service_cb)
        self.get_logger().info("Mask based Grasp Prediction Service is ready.")

        # rospy.Service('~predict', Grasp2DPrediction, self.service_cb)

    def service_cb(self, request:Grasp2DPrediction.Request, response:Grasp2DPrediction.Response):
        print("Request received")
        depth = bridge.imgmsg_to_cv2(request.depth_image)
        
        grasp_mask = GraspMask(min(depth.shape))
        x, y, angle, width = grasp_mask.get_grasp(depth)

        print("Got grasp")
 
        g = response.best_grasp
        # Scale detection for correct 3D transformation
        g.px = x
        g.py = y
        g.angle = angle + 1.57
        g.width = float(width)
        g.quality = 1.0

        # response.best_grasp.px = 100
        # response.best_grasp.py = 100
        # response.best_grasp.angle = 0+1.57
        # response.best_grasp.width = 0.2
        # response.best_grasp.quality = 1.0
        # response.success = True
        # # print("returning", response)
        # resp = Grasp2DPrediction.Response()
        # print(resp)
        # resp.best_grasp.px = x
        # resp.best_grasp.py = y
        # resp.best_grasp.angle = angle+1.57
        # resp.best_grasp.width = width
        # resp.best_grasp.quality = 1

        # self.get_logger().info(f"Grasp in Image frame: {g.px}, {g.py}, {g.angle} ({depth.shape})")
        # print(f"Grasp in Image frame: {resp.best_grasp.px}, {resp.best_grasp.py}, {resp.best_grasp.angle} ({depth.shape})")\
        print("returning")

        return response
  
def main(args=None):
    rclpy.init(args=args)
    node = GraspService()
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    try:
        # executor.spin()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()

