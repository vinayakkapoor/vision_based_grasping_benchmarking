import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_matrix, quaternion_from_matrix, euler_matrix

import numpy as np

from isaac_ros2_messages.srv import SetPrimAttribute
from franka_kinematics import *


ycb_object_name_to_prim_path = {
    'banana': '/World/_11_banana/_11_banana',
    'large_clamp': '/World/_51_large_clamp/_51_large_clamp',
    'mustard_bottle': '/World/_06_mustard_bottle/_06_mustard_bottle',
    'medium_clamp': '/World/medium_clamp/medium_clamp/medium_clamp',
    'extra_large_clamp': '/World/_52_extra_large_clamp/_52_extra_large_clamp',
    'pear': '/World/pear/pear/pear',
    'screwdriver': '/World/screwdriver/flat_screwdriver/flat_screwdriver',
    'strawberry': '/World/strawberry/strawberry/strawberry_003',
    'tennis_ball': '/World/tennis_ball/tennis_ball/tennis_ball',
    'mug': '/World/_25_mug/_25_mug'
}
# Must be float values
ycb_object_inactive_pose = {
    'banana': [0.0, 1.0, 0.1],
    'tennis_ball': [0.2, 1.0, 0.1],
    'mustard_bottle': [0.4, 1.0, 0.1],
    'large_clamp': [0.6, 1.0, 0.1],
    'extra_large_clamp': [0.8, 1.0, 0.1],
    'medium_clamp': [1.0, 1.0, 0.1],
    'pear': [1.2, 1.0, 0.1],
    'screwdriver': [1.4, 1.0, 0.1],
    'strawberry': [1.6, 1.0, 0.1],
    'mug': [1.8, 1.0, 0.1],
}
ycb_object_initial_rotation = {
    'banana': [0.0, 0.0, 0.0],
    'tennis_ball': [0, 0, 0],
    'mustard_bottle': [-90, 0, 0],
    'large_clamp': [0, 0, 0],    
    'extra_large_clamp': [0, 0, 0],
    'medium_clamp': [0, 0, 0],    
    'pear': [0, 0, 0],
    'screwdriver': [0, 0, 0],
    'strawberry': [0, 0, 0],    
    'mug': [-90, 0, 0],
}

class IsaacSimROS2Adapter:
    def __init__(self, ros2_node:Node):
        self.ros2_node:Node = ros2_node
        # self.cb_group = ReentrantCallbackGroup()
        self.set_prim_attribute_client = self.ros2_node.create_client(
            SetPrimAttribute,
            '/set_prim_attribute',
            callback_group=ros2_node.reentrant_callback_group
        )

        self.joint_state_publisher = self.ros2_node.create_publisher(
            JointState, 
            "joint_command", 10,
            callback_group=ros2_node.reentrant_callback_group
        )

        self.finger_dist_sub = self.ros2_node.create_subscription(
            TFMessage,
            '/tf_fingers',
            self.finger_tf_cb,
            10,
            callback_group=ros2_node.reentrant_callback_group
        )

        self.gripper_width = self.ros2_node.get_parameter('gripper_width').value
        self.attached = True 
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = joint_names
    
    def finger_tf_cb(self, msg:TFMessage):
        for transform in msg.transforms:
            if transform.header.frame_id == "panda_rightfinger" and transform.child_frame_id == "panda_leftfinger":
                finger_distance = transform.transform.translation.y
                if finger_distance < self.gripper_width - 0.015 and finger_distance > 0.01:
                    self.attached = True
                else:
                    self.attached = False
                break

    def spawn_model(self, object_name:str, object_pose:Pose)->bool:
        try:
            object_prim_path = ycb_object_name_to_prim_path[object_name]
        except KeyError as k_err:
            self.ros2_node.get_logger().error("invalid object name")
            return False
        
        init_rot_deg = ycb_object_initial_rotation[object_name]
        T_init = np.eye(4)
        R = euler_matrix(
            np.deg2rad(init_rot_deg[0]),
            np.deg2rad(init_rot_deg[1]),
            np.deg2rad(init_rot_deg[2]),
            'sxyz'
        )[:3, :3]
        self.ros2_node.get_logger().info(f"{R}")
        T_init[:3,:3] = R
        transformed_pose = self.rotate_pose(T_init, object_pose)
        request_orientation = SetPrimAttribute.Request()
        request_orientation.path = object_prim_path
        request_orientation.attribute = 'xformOp:orient'
        request_orientation.value = f"""[{transformed_pose.orientation.w}, 
                                     {transformed_pose.orientation.x}, 
                                     {transformed_pose.orientation.y}, 
                                     {transformed_pose.orientation.z}]"""
        #print("calling orientation attribute")
        future_orientation = self.set_prim_attribute_client.call_async(request_orientation)

        request_translation = SetPrimAttribute.Request()
        request_translation.path = object_prim_path
        request_translation.attribute = 'xformOp:translate'
        request_translation.value = f"""[{object_pose.position.x}, 
                                     {object_pose.position.y},
                                     {object_pose.position.z}]"""
        future_translation = self.set_prim_attribute_client.call_async(request_translation)

        # rclpy.spin_until_future_complete(self.ros2_node, future_orientation)
        # rclpy.spin_until_future_complete(self.ros2_node, future_translation)
        self.ros2_node._wait_for_future(future_orientation)
        self.ros2_node._wait_for_future(future_translation)

        return True

    def delete_model(self, object_name:str)->bool:
        inactive_pose = Pose()
        inactive_pose.position.x = ycb_object_inactive_pose[object_name][0]
        inactive_pose.position.y = ycb_object_inactive_pose[object_name][1]
        inactive_pose.position.z = ycb_object_inactive_pose[object_name][2]
        self.spawn_model(object_name, inactive_pose)
        # TODO: update visibility

        return True
    
    def set_joint_states(self, joint_positions=None, joint_velocities=None, joint_efforts=None):        
        self.joint_state_msg.header.stamp = self.ros2_node.get_clock().now().to_msg()
        if joint_positions is not None:
            self.joint_state_msg.position = joint_positions
        if joint_velocities is not None:
            self.joint_state_msg.velocity = joint_velocities
        if joint_efforts is not None:
            self.joint_state_msg.effort = joint_efforts
        self.joint_state_publisher.publish(self.joint_state_msg)

    
    def rotate_pose(self, T: np.ndarray, pose: Pose) -> Pose:        
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        R_pose = quaternion_matrix(quat)  # 4x4 matrix

        # 3. Apply rotation: T * R_pose
        R_transformed = R_pose @ T

        # 4. Extract transformed quaternion
        quat_transformed = quaternion_from_matrix(R_transformed)

        # 5. Create new Pose
        new_pose = Pose()
        new_pose.orientation.x = quat_transformed[0]
        new_pose.orientation.y = quat_transformed[1]
        new_pose.orientation.z = quat_transformed[2]
        new_pose.orientation.w = quat_transformed[3]

        return new_pose
        
