#!/usr/bin/env python

import rospy
from moveit_adapter_module.grasping import Gripper
from moveit_adapter_module.eef_control import MoveGroupControl
from moveit_adapter.srv import EndEffectorWaypoint, GripperCommand, CurrentPose, SetJointVelocity, StopMovement, MoveJointRelative

class MoveitAdapter:
    def __init__(self):
        sim_mode = rospy.get_param('sim_mode', False)
    
        self.gripper = Gripper(sim_mode=sim_mode)
        self.moveit_control = MoveGroupControl(gripper_as_eef=True)
        
        rospy.Service('moveit_adapter/grasp', GripperCommand, self.grasp_service)
        rospy.Service('moveit_adapter/cartesian_path', EndEffectorWaypoint, self.cartesian_path_service)
        rospy.Service('moveit_adapter/vanilla', EndEffectorWaypoint, self.vanilla_path_service)
        rospy.Service('moveit_adapter/get_current_pose', CurrentPose, self.get_current_pose_service)
        rospy.Service('moveit_adapter/set_joint_velocity', SetJointVelocity, self.set_joint_velocity)
        rospy.Service('moveit_adapter/stop', StopMovement, self.stop_movement)
        rospy.Service('moveit_adapter/move_joint', MoveJointRelative, self.move_joint)
        
    def cartesian_path_service(self, req):
        self.moveit_control.follow_cartesian_path([[req.x, req.y, req.z, req.roll, req.pitch, req.yaw]])
        return True
        
    def vanilla_path_service(self, req):
        self.moveit_control.go_to_pose_goal(req.x, req.y, req.z, req.roll, req.pitch, req.yaw)
        return True
        
    def grasp_service(self, req):
        self.gripper.grasp(req.width)
        return True
    
    def get_current_pose_service(self, req):
        return self.moveit_control.get_current_pose()

    def set_joint_velocity(self, req):
        self.moveit_control.set_joint_velocity(req.joint_velocity)
        return True

    def stop_movement(self, req):
        self.moveit_control.stop_robot()
        return True

    def move_joint(self, req):
        joint = req.joint
        ang_disp = req.ang_disp

        ang_disp_joints = [0, 0, 0, 0, 0, 0, 0]
        ang_disp_joints[joint] = ang_disp_joints[joint] + ang_disp 

        joint_states = self.moveit_control.get_current_joint_states()
        self.moveit_control.go_to_joint_state(joint_states[0] + ang_disp_joints[0], joint_states[1] + ang_disp_joints[1], 
                                              joint_states[2] + ang_disp_joints[2], joint_states[3] + ang_disp_joints[3], 
                                              joint_states[4] + ang_disp_joints[4], joint_states[5] + ang_disp_joints[5], 
                                              joint_states[6] + ang_disp_joints[6])    
        return True

if __name__ == "__main__":
    rospy.init_node('moveit_adapter_node')
    moveit_adapter = MoveitAdapter()
    rospy.spin()