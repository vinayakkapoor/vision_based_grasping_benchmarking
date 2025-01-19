#!/usr/bin/env python3

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace
from math import pi

def pick_and_place():
    pick_and_place = PickAndPlace(gripper_offset=0.105, intermediate_z_stop=0.7)
    
    pick_and_place.setDropPose(x=0.0, y=0.4, z=0.4, roll=0, pitch=pi, yaw=0)
    pick_and_place.setPickPose(x=0.45, y=0.0, z=0.1, roll=pi, pitch=0, yaw=-pi/4)
    pick_and_place.setGripperPose(width=0.0)
    
    # pick_and_place.execute_pick_and_place()
    pick_and_place.execute_pick_up()
    # pick_and_place.execute_place()
    # pick_and_place.execute_cartesian_pick_and_place()
    # pick_and_place.execute_cartesian_pick_up()
    # pick_and_place.execute_cartesian_place()

if __name__ == "__main__":
    rospy.init_node("test_pnp")
    pick_and_place()