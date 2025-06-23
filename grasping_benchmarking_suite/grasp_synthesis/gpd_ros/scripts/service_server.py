#!/usr/bin/env python

import rospy
import numpy as np
from copy import deepcopy 

from gpd_ros.srv import GraspPrediction, GraspPredictionResponse
from gpd_ros.msg import CloudSamples, GraspConfigList
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Pose

class GraspService:
    def __init__(self):
        rospy.Service('~predict', GraspPrediction, self.service_cb)
        self.gpd_pub = rospy.Publisher('/cloud_stitched', CloudSamples, queue_size=1)
        self.pcl_sub = rospy.Subscriber('/camera/depth/color/roi_points', PointCloud2, self.pcl_cb)
        self.grasp_config_list_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.grasp_config_list_cb)
        self.curr_graps_config_list = None

    def pcl_cb(self, data: PointCloud2):
        self.current_pcl = data
    
    def grasp_config_list_cb(self, data: GraspConfigList):
        self.curr_graps_config_list = data

    def service_cb(self, request):        
        response = GraspPredictionResponse()
        pcl_msg = deepcopy(self.current_pcl)
        
        cloud_samples_msg = CloudSamples()
        cloud_samples_msg.cloud_sources.cloud = pcl_msg
        cloud_samples_msg.cloud_sources.view_points.append(Point(x=0, y=0, z=0))

        rospy.loginfo("Publishing cloud")
        self.gpd_pub.publish(cloud_samples_msg)

        while self.curr_graps_config_list is None:
            rospy.sleep(0.5)
            rospy.loginfo("Waiting for gpd grasps")
        
        
        pose = Pose()
        pose.position = self.curr_graps_config_list.grasps[0].position # curr_graps_config_list is already sorted by score

        response.best_grasp.pose = pose

        self.curr_graps_config_list = None

        return response

if __name__ == '__main__':
    rospy.init_node('gpd_grasp_service')
    GraspService()
    rospy.spin()


