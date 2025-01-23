#!/usr/bin/env python3

import rospy
from grasp_transform_module.image_to_camera_frame import ImageToCameraFrame

if __name__ == "__main__":
    rospy.init_node('image_to_camera_node')

    sim_mode = rospy.get_param("sim_mode")
    enable_crop = rospy.get_param("enable_crop")
    cam_to_world_module = ImageToCameraFrame(sim_mode=sim_mode, crop=enable_crop)

    rospy.spin()
