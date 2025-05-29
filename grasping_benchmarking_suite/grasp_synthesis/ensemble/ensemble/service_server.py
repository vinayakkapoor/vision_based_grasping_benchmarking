#!/usr/bin/env python3
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
print(current_dir)
sys.path.append(current_dir)
sys.path.append(os.path.join(current_dir, '..', 'ensemble'))

import rclpy 
from rclpy.node import Node
import numpy as np 
import cv_bridge as cvb

bridge = cvb.CvBridge()
import os


from ensemble_module.ensemble import Ensemble
from grasp_interfaces.srv import Grasp2DPrediction

from utils import process_depth_image, process_rgb


class GraspService(Node):
    def __init__(self):
        super().__init__('grasp_service')
        # in ros2 no need to add ~predict to append to node name
        self.srv = self.create_service(Grasp2DPrediction,'/ensemble_grasp_service/predict',self.service_cb) 
        self.ensemble = Ensemble()
        trained_model_dir = os.path.join(os.path.dirname(current_dir), 'src/ensemble_module/trained_models')
        self.ensemble.init_model('expert1',
                                 trained_model_dir + '/ggcnn_epoch_23_cornell',
                                 'cpu')
        self.ensemble.init_model('expert2',
                                 trained_model_dir + '/ggcnn_epoch_34_jacquard',
                                 'cpu')
        self.ensemble.init_model('expert3',
                                 trained_model_dir + '/gpnn_epoch_40_cornell',
                                 'cpu')
        self.ensemble.init_model('expert4',
                                 trained_model_dir + '/gpnn_epoch_48_jacquard',
                                 'cpu')
        self.ensemble.init_model('expert5',
                                 trained_model_dir + '/grcnn_epoch_30_cornell',
                                 'cpu')
        self.ensemble.init_model('expert6',
                                 trained_model_dir + '/grcnn_epoch_48_jacquard',
                                 'cpu')
        self.ensemble.init_model('expert7',
                                 trained_model_dir + '/HiFormer_trial_2_fold_3_ep_11_iou_0.9610_lr_5p1e-04_bs_8.pth',
                                 'cpu', True, in_channels=4)
        # self.ensemble.init_model('ensemble',
        #                          trained_model_dir + '/ensemble_hiformer_trial_0_epoch_16_iou_0.9661_lr_3p7e-04_bs_16.pth',
        #                          'cpu')
        self.ensemble.init_model('ensemble',
                                 trained_model_dir + '/HiFormer_trial_3_fold_4_ep_07_iou_0.9480_lr_1p1e-04_bs_32.pth',
                                 'cpu', is_hiformer=True, in_channels=13)    
        # self.ensemble.init_model('ensemble',
        #                          trained_model_dir + '/HiFormer_trial_2_fold_3_ep_11_iou_0.9610_lr_5p1e-04_bs_8.pth',
        #                          'cpu')   
        #        
        

    def service_cb(self,request,response):
        depth = bridge.imgmsg_to_cv2(request.depth_image)
        rgb = bridge.imgmsg_to_cv2(request.rgb_image)
        
        depth_crop, depth_nan_mask = process_depth_image(depth, depth.shape[0], 300, return_mask=True, crop_y_offset=0)
        rgb_crop = process_rgb(rgb, crop_size=depth.shape[0], out_size=300, crop_y_offset=0)

        quality, angle, width_img = self.ensemble.predict(rgb_crop, depth_crop)
        # quality, angle, width_img = predict(depth_crop, process_depth=False, depth_nan_mask=depth_nan_mask, filters=(2.0, 2.0, 2.0))

        x, y = np.unravel_index(np.argmax(quality), quality.shape)
        ang = angle[x][y]

        response = Grasp2DPrediction.Response()
        g = response.best_grasp

        # # Scale detection for correct 3D transformation
        g.px = int(x*depth.shape[0]/300.0)
        g.py = int(y*depth.shape[0]/300.0 + (depth.shape[1] - depth.shape[0])/2)
        g.angle = float(ang)
        g.width = float(width_img[x][y]*depth.shape[0]/300)
        g.quality = float(quality[x][y])

        self.get_logger().info(str(g.px))
        self.get_logger().info(str(g.py))
        self.get_logger().info(str(g.angle))
        self.get_logger().info(str(g.width))
        self.get_logger().info(str(g.quality))

        return response



def main():
    "main function"
    rclpy.init()
    grasp_service = GraspService()
    rclpy.spin(grasp_service)
    rclpy.shutdown()



if __name__ == '__main__':
    main()


