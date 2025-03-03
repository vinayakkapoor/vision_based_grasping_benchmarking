from copy import deepcopy
from math import pi

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup

from grasp_interfaces.srv import (
    EndEffectorWaypoint, 
    GripperCommand, 
    CurrentPose, 
    SetJointVelocity, 
    MoveJointRelative
)

class PickAndPlace:
    def __init__(self, node, gripper_offset=0.05, intermediate_z_stop=0.5):
        self.node = node
        self.gripper_offset = gripper_offset
        self.intermediate_z_stop = intermediate_z_stop
        self.scan_pose = [0.0, 0.3, 0.6, 0.0, pi, 0.0]
        self.home_pose = [0.0, 0.3, 0.6, 0.0, pi, 0.0]
        self.pick_pose = None
        self.place_pose = None
        self.gripper_pose = None
        self.angle_offset = 0.0
        self.stop_above_destination = 0.05

        # Get parameters
        self.default_velocity = self.node.get_parameter(
            'robot_default_velocity').value
        self.approach_velocity = self.node.get_parameter(
            'robot_approach_velocity').value

        # Create service clients with a ReentrantCallbackGroup to allow parallel execution
        self.cb_group = ReentrantCallbackGroup()
        self.cartesian_client = self.node.create_client(
            EndEffectorWaypoint, 'moveit_adapter/cartesian_path',
            callback_group=self.cb_group
        )
        self.vanilla_client = self.node.create_client(
            EndEffectorWaypoint, 'moveit_adapter/vanilla',
            callback_group=self.cb_group
        )
        self.gripper_client = self.node.create_client(
            GripperCommand, 'moveit_adapter/grasp',
            callback_group=self.cb_group
        )
        self.current_pose_client = self.node.create_client(
            CurrentPose, 'moveit_adapter/get_current_pose',
            callback_group=self.cb_group
        )
        self.set_joint_velocity_client = self.node.create_client(
            SetJointVelocity, 'moveit_adapter/set_joint_velocity',
            callback_group=self.cb_group
        )
        self.move_joint_client = self.node.create_client(
            MoveJointRelative, '/moveit_adapter/move_joint',
            callback_group=self.cb_group
        )

    def setPickPose(self, x, y, z, roll, pitch, yaw):
        self.pick_pose = [x, y, z, roll, pitch, yaw + self.angle_offset]
    
    def setDropPose(self, x, y, z, roll, pitch, yaw):
        self.drop_pose = [x, y, z, roll, pitch, yaw + self.angle_offset]
    
    def setHomePose(self, x=0.0, y=0.3, z=0.6, roll=0.0, pitch=pi, yaw=0.0):
        self.home_pose = [x, y, z, roll, pitch, yaw + self.angle_offset]

    def setScanPose(self, x=0.0, y=0.3, z=0.6, roll=0.0, pitch=pi, yaw=0.0):
        self.scan_pose = [x, y, z, roll, pitch, yaw + self.angle_offset]

    def setGripperPose(self, width=0.0):
        self.gripper_pose = width
        
    def call_cartesian_service(self, waypoint):
        if not self.cartesian_client.service_is_ready():
            self.node.get_logger().warn('Cartesian service not available')
            return

        req = EndEffectorWaypoint.Request()
        req.x = waypoint[0]
        req.y = waypoint[1]
        req.z = waypoint[2]
        req.roll = waypoint[3]
        req.pitch = waypoint[4]
        req.yaw = waypoint[5]

        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().debug('Cartesian move succeeded')
        else:
            self.node.get_logger().error('Cartesian move failed')

    def call_vanilla_service(self, waypoint):
        if not self.vanilla_client.service_is_ready():
            self.node.get_logger().warn('Vanilla service not available')
            return

        req = EndEffectorWaypoint.Request()
        req.x = waypoint[0]
        req.y = waypoint[1]
        req.z = waypoint[2]
        req.roll = waypoint[3]
        req.pitch = waypoint[4]
        req.yaw = waypoint[5]

        future = self.vanilla_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().debug('Vanilla move succeeded')
        else:
            self.node.get_logger().error('Vanilla move failed')

    def call_gripper_service(self, width):
        if not self.gripper_client.service_is_ready():
            self.node.get_logger().warn('Gripper service not available')
            return

        req = GripperCommand.Request()
        req.width = float(width)

        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().debug('Gripper move succeeded')
        else:
            self.node.get_logger().error('Gripper move failed')

    def call_get_current_pose_service(self):
        if not self.current_pose_client.service_is_ready():
            self.node.get_logger().warn('Current pose service not available')
            return []

        req = CurrentPose.Request()
        future = self.current_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result() is not None:
            return [
                future.result().x,
                future.result().y,
                future.result().z,
                future.result().roll,
                future.result().pitch,
                future.result().yaw
            ]
        else:
            self.node.get_logger().error('Current pose service call failed')
            return []

    def call_set_joint_velocity_service(self, velocity):
        if not self.set_joint_velocity_client.service_is_ready():
            self.node.get_logger().warn('Set velocity service not available')
            return

        req = SetJointVelocity.Request()
        req.joint_velocity = float(velocity)

        future = self.set_joint_velocity_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().debug('Velocity set succeeded')
        else:
            self.node.get_logger().error('Velocity set failed')

    def call_move_joint_service(self, joint, ang_disp):
        if not self.move_joint_client.service_is_ready():
            self.node.get_logger().warn('Move joint service not available')
            return

        req = MoveJointRelative.Request()
        req.joint = str(joint)
        req.ang_disp = float(ang_disp)

        future = self.move_joint_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().debug('Joint move succeeded')
        else:
            self.node.get_logger().error('Joint move failed')

    def generate_waypoints(self, destination_pose, action, interpolate=False, interpolate_steps=3):
        waypoints = []

        if action == 0:
            current_pose = self.call_get_current_pose_service()
            current_pose_ = deepcopy(destination_pose)
            current_pose_[0] = self.scan_pose[0]
            current_pose_[1] = self.scan_pose[1]
            current_pose_[2] = self.scan_pose[2]
            current_pose_[3] = self.scan_pose[3]
            current_pose_[4] = self.scan_pose[4]
            current_pose_[5] = self.scan_pose[5]
            waypoints.append(current_pose_)

        if action == 1:
            current_pose_ = deepcopy(destination_pose)
            current_pose_[2] = self.intermediate_z_stop
            waypoints.append(current_pose_)

            if interpolate:
                for i in range(1, interpolate_steps):
                    dest = deepcopy(destination_pose)
                    dest[2] = self.intermediate_z_stop - (
                        self.intermediate_z_stop - (dest[2] + self.stop_above_destination + self.gripper_offset)
                    ) * i / interpolate_steps
                    waypoints.append(dest)

            dest = deepcopy(destination_pose)
            dest[2] += self.gripper_offset + self.stop_above_destination
            waypoints.append(dest)

        if action == 2:
            intermediate_pose = deepcopy(destination_pose)
            intermediate_pose[2] = self.intermediate_z_stop
            waypoints.append(intermediate_pose)

        if action == 3:
            current_pose = self.call_get_current_pose_service()
            current_pose_ = deepcopy(destination_pose)
            current_pose_[0] = current_pose[0]
            current_pose_[1] = current_pose[1]
            current_pose_[2] = self.intermediate_z_stop
            waypoints.append(current_pose_)

            dest = deepcopy(destination_pose)
            dest[2] += self.gripper_offset
            waypoints.append(dest)

        if action == 4:
            dest = deepcopy(destination_pose)
            dest[2] += self.gripper_offset
            waypoints.append(dest)

        return waypoints
    
    def execute_cartesian_pick_and_place(self):
        self.execute_cartesian_pick_up()
        self.execute_cartesian_place()

    def execute_pick_and_place(self):
        self.execute_pick_up()
        self.execute_place()

    def execute_cartesian_pick_up(self):        
        self.call_gripper_service(0.1)
        self.node.get_clock().sleep_for(Duration(seconds=2))
        
        waypoints = self.generate_waypoints(self.pick_pose, 1)
        for waypoint in waypoints:
            self.call_cartesian_service(waypoint)

        waypoints = self.generate_waypoints(self.pick_pose, 4)
        for waypoint in waypoints:
            self.call_cartesian_service(waypoint)

        self.call_gripper_service(self.gripper_pose)
        self.node.get_clock().sleep_for(Duration(seconds=3))

        waypoints = self.generate_waypoints(self.pick_pose, 2)
        for waypoint in waypoints:
            self.call_cartesian_service(waypoint)
    
    def execute_pick_up(self):
        self.call_set_joint_velocity_service(self.default_velocity)
        self.call_gripper_service(0.1)
        self.node.get_clock().sleep_for(Duration(seconds=2))        

        waypoints = self.generate_waypoints(self.pick_pose, 1)        
        for waypoint in waypoints:
            self.call_vanilla_service(waypoint)

        self.call_set_joint_velocity_service(self.approach_velocity)
        waypoints = self.generate_waypoints(self.pick_pose, 4)
        for waypoint in waypoints:
            self.call_vanilla_service(waypoint)

        self.call_gripper_service(self.gripper_pose)
        self.node.get_clock().sleep_for(Duration(seconds=3))

        self.call_set_joint_velocity_service(self.default_velocity)
        waypoints = self.generate_waypoints(self.pick_pose, 2)        
        for waypoint in waypoints:
            self.call_vanilla_service(waypoint)
        
    def execute_place(self):
        self.call_set_joint_velocity_service(self.default_velocity)

        waypoints = self.generate_waypoints(self.pick_pose, 1)        
        for waypoint in waypoints:
            self.call_vanilla_service(waypoint)

        self.call_set_joint_velocity_service(self.approach_velocity)
        waypoints = self.generate_waypoints(self.pick_pose, 4)
        for waypoint in waypoints:
            self.call_vanilla_service(waypoint)

        self.call_gripper_service(0.1)
        self.node.get_clock().sleep_for(Duration(seconds=3))
    
    def execute_cartesian_place(self):
        waypoints = self.generate_waypoints(self.drop_pose, 3)
        for waypoint in waypoints:
            self.call_cartesian_service(waypoint)

        self.call_gripper_service(0.1)
        self.node.get_clock().sleep_for(Duration(seconds=3))        

    def reach_scanpose(self):
        self.call_set_joint_velocity_service(self.default_velocity)
        waypoints = self.generate_waypoints(self.scan_pose, 0)

        for waypoint in waypoints:
            self.call_vanilla_service(waypoint)

    def reach_cartesian_scanpose(self):
        waypoints = self.generate_waypoints(self.scan_pose, 0)
        for waypoint in waypoints:
            self.call_cartesian_service(waypoint)