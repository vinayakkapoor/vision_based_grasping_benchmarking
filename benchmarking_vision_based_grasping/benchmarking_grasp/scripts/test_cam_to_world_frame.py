#!/usr/bin/env python3

import rospy
from grasp_transform_module.camera_to_world_frame import CameraToWorldFrame

if __name__ == "__main__":
    rospy.init_node('camera_to_world_node')

    sim_mode = rospy.get_param("sim_mode")
    cam_to_world_module = CameraToWorldFrame(sim_mode=sim_mode)

    rospy.spin()
