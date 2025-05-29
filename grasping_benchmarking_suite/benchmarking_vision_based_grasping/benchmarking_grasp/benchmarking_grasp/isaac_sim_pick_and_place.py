import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from math import pi
import time
from copy import deepcopy
from isaac_sim_ros2_adapter import IsaacSimROS2Adapter
from franka_kinematics import joint_names
from rclpy.node import Node
import rclpy

class IsaacPickAndPlace:
    def __init__(self, node: Node, gripper_offset=0.05, intermediate_z_stop=0.5, sim_adapter: IsaacSimROS2Adapter = None):
        self.node = node
        self.logger = self.node.get_logger()

        if sim_adapter is None:
            self.logger.info("Creating default IsaacSimROS2Adapter")
            self.sim_adapter = IsaacSimROS2Adapter(self.node)
        else:
            self.sim_adapter = sim_adapter

        try:
            self.robot = rtb.models.Panda()
            self.logger.info("Loaded Panda model from roboticstoolbox.")

            if self.robot.qlim.shape[1] != self.robot.n:
                 self.logger.warn(f"Robot model qlim shape {self.robot.qlim.shape} inconsistent with n={self.robot.n}. Joint limit checks might fail.")
                 self.qlim = None
            else:
                 self.qlim = self.robot.qlim

            self.current_q = deepcopy(self.robot.qr)

            if self.robot.n < len(joint_names):
                 self.logger.info(f"RTB model has {self.robot.n} joints, adding gripper joints to state vector (total {len(joint_names)})")
                 gripper_open_val = 0.04
                 self.current_q = np.concatenate((self.current_q, [gripper_open_val, gripper_open_val]))

            if len(self.current_q) != len(joint_names):
                 raise ValueError(f"Mismatch between final q length ({len(self.current_q)}) and joint_names length ({len(joint_names)})")

            self.num_robot_joints = self.robot.n

        except ValueError as e:
            self.logger.error(f"Failed to load Panda model: {e}")
            self.logger.error("Check if 'roboticstoolbox-python' and its dependencies are installed correctly.")
            raise RuntimeError("Could not initialize robot model.") from e

        self.gripper_offset = gripper_offset
        self.intermediate_z_stop = intermediate_z_stop
        self.stop_above_destination = 0.05
        self.angle_offset = 0.0

        self.benchmarking_velocity = self.node.get_parameter_or('robot_benchmarking_velocity', rclpy.Parameter('robot_benchmarking_velocity', rclpy.Parameter.Type.DOUBLE, 0.5)).value
        self.default_velocity = self.node.get_parameter_or('robot_default_velocity', rclpy.Parameter('robot_default_velocity', rclpy.Parameter.Type.DOUBLE, 0.3)).value
        self.approach_velocity = self.node.get_parameter_or('robot_approach_velocity', rclpy.Parameter('robot_approach_velocity', rclpy.Parameter.Type.DOUBLE, 0.1)).value

        self.gripper_indices = [len(joint_names) - 2, len(joint_names) - 1]
        self.gripper_open_value = 0.2
        self.gripper_closed_value = 0.0
        self.gripper_force_close_value = -0.02
        self.gripper_force_close_effort = -0.02
        self.is_gripper_open = True

        self.home_q = deepcopy(self.robot.qr)
        if len(self.home_q) < len(joint_names):
             self.home_q = np.concatenate((self.home_q, [self.gripper_open_value, self.gripper_open_value]))
        self.home_pose_se3 = self.robot.fkine(self.home_q[:self.num_robot_joints])

        self.scan_pose_se3 = self._list_to_se3([0.0, 0.3, 0.6, 0.0, pi, 0.0])
        self.pick_pose_se3 = None
        self.drop_pose_se3 = None
        self.target_gripper_width = self.gripper_open_value

        self.trajectory_time_step = 0.02

        self.logger.info("IsaacPickAndPlace initialized.")
        self.sim_adapter.set_joint_states(self.current_q.tolist())
        time.sleep(0.5)

    def _list_to_se3(self, pose_list):
        if pose_list is None or len(pose_list) != 6:
            self.logger.error(f"Invalid pose list format: {pose_list}")
            return None
        return SE3(pose_list[0:3]) * SE3.RPY(pose_list[3], pose_list[4], pose_list[5], order='zyx')

    def _calculate_ik(self, target_pose_se3: SE3, q_initial_guess=None):
        if target_pose_se3 is None:
            self.logger.error("Target pose is None, cannot calculate IK.")
            return None, False

        if q_initial_guess is None:
            q_initial_guess = self.current_q[:self.num_robot_joints]

        sol = self.robot.ikine_LM(
            target_pose_se3,
            q0=q_initial_guess,
            joint_limits=True,
            slimit=150,
            )

        if sol.success:
            final_q = sol.q
            if self.qlim is not None:
                lower_limits = self.qlim[0, :]
                upper_limits = self.qlim[1, :]
                if not np.all((final_q >= lower_limits) & (final_q <= upper_limits)):
                     self.logger.warn(f"IK solution found but violates joint limits. Solution: {final_q}, Limits L: {lower_limits}, U: {upper_limits}")
                     return None, False

            self.logger.debug(f"IK solution found: {final_q}")
            full_q = np.concatenate((final_q, self.current_q[self.gripper_indices]))
            return full_q, True
        else:
            self.logger.warn(f"IK solution not found for target pose: {target_pose_se3.t}, residual: {sol.residual}")
            return None, False

    def _execute_trajectory(self, trajectory_q, is_gripper_move=False):
        if trajectory_q is None or len(trajectory_q) == 0:
            self.logger.warn("Empty trajectory received, skipping execution.")
            return False

        num_steps = len(trajectory_q)
        self.logger.debug(f"Executing trajectory with {num_steps} steps.")

        for i, q_step in enumerate(trajectory_q):
            q_step_np = np.asarray(q_step)
            if len(q_step_np) != len(joint_names):
                 self.logger.error(f"Trajectory step {i} has wrong length: {len(q_step_np)}, expected {len(joint_names)}")
                 return False

            if self.is_gripper_open:
                self.sim_adapter.set_joint_states(q_step_np.tolist())
            else:
                joint_efforts = np.ones(len(q_step_np.tolist()))*25
                joint_efforts[-1] = -self.gripper_force_close_effort
                joint_efforts[-2] = self.gripper_force_close_effort
                # self.sim_adapter.set_joint_states(q_step_np.tolist(), joint_efforts=joint_efforts.tolist())
                self.sim_adapter.set_joint_states(q_step_np.tolist())
            self.current_q = q_step_np

            time.sleep(self.trajectory_time_step)

        self.logger.debug("Trajectory execution finished.")
        time.sleep(0.2)
        return True

    def _generate_trajectory(self, q_start, q_end, velocity, is_cartesian=False, T_start=None, T_end=None):
        if q_start is None:
             self.logger.error("Cannot generate trajectory with None start configuration (q_start).")
             return None

        q_start = np.asarray(q_start)

        if is_cartesian:
            if T_start is None or T_end is None:
                self.logger.error("Cartesian trajectory requires start/end SE3 poses (T_start, T_end).")
                return None
        else:
            if q_end is None:
                 self.logger.error("Cannot generate joint trajectory with None end configuration (q_end).")
                 return None
            q_end = np.asarray(q_end)
            if len(q_start) != len(q_end):
                 self.logger.error(f"Mismatched q_start ({len(q_start)}) and q_end ({len(q_end)}) lengths.")
                 return None

        if is_cartesian:
            distance = np.linalg.norm(T_end.t - T_start.t)
            if distance < 1e-4:
                 self.logger.debug("Cartesian move too small, generating minimal trajectory.")
                 q_end_final_arm, ik_success = self._calculate_ik(T_end, q_initial_guess=q_start[:self.num_robot_joints])
                 if not ik_success:
                      self.logger.error(f"IK failed even for the target pose of a small Cartesian move. Target: {T_end.t}")
                      return None
                 q_end_final = np.concatenate((q_end_final_arm[:self.num_robot_joints], q_start[self.gripper_indices]))
                 return [q_start, q_end_final]

            duration = max(distance / velocity, self.trajectory_time_step * 2)
            num_steps = max(int(duration / self.trajectory_time_step), 1) + 1
            self.logger.debug(f"Generating Cartesian trajectory: Dist={distance:.3f}, Vel={velocity:.2f}, Duration={duration:.2f}, Steps={num_steps}")

            try:
                time_vector = np.linspace(0, duration, num_steps)
                T_traj = rtb.ctraj(T_start, T_end, t=time_vector)
            except Exception as e:
                 self.logger.error(f"rtb.ctraj failed: {e}")
                 return None

            q_traj = []
            last_valid_q_arm = q_start[:self.num_robot_joints]
            current_gripper_q = q_start[self.gripper_indices]

            for i, T_step in enumerate(T_traj):
                if not isinstance(T_step, SE3):
                    self.logger.error(f"Invalid type returned by ctraj at step {i}: {type(T_step)}. Aborting.")
                    return None

                q_ik_full, success = self._calculate_ik(T_step, q_initial_guess=last_valid_q_arm)
                if success:
                    q_ik_arm = q_ik_full[:self.num_robot_joints]
                    full_q_step = np.concatenate((q_ik_arm, current_gripper_q))
                    q_traj.append(full_q_step)
                    last_valid_q_arm = q_ik_arm
                else:
                    self.logger.warn(f"IK failed at step {i}/{len(T_traj)-1} of Cartesian trajectory (Target T: {T_step.t}). Aborting generation.")
                    return None
            return q_traj

        else:
            joint_diff = q_end - q_start
            distance = np.linalg.norm(joint_diff)

            if distance < 1e-5:
                 self.logger.debug("Joint move too small, generating minimal trajectory.")
                 return [q_start, q_end]

            duration = max(distance / velocity, self.trajectory_time_step * 2)
            num_steps = max(int(duration / self.trajectory_time_step), 1) + 1
            self.logger.debug(f"Generating Joint trajectory: Dist={distance:.3f}, Vel={velocity:.2f} (approx avg), Duration={duration:.2f}, Steps={num_steps}")

            time_vector = np.linspace(0, duration, num_steps)

            try:
                traj_result = rtb.jtraj(q_start, q_end, time_vector)
                q_traj_np = traj_result.q
            except Exception as e:
                self.logger.error(f"rtb.jtraj failed: {e}")
                return None

            if q_traj_np is None or len(q_traj_np) == 0 or q_traj_np.shape[1] != len(q_start):
                 self.logger.error(f"jtraj returned invalid trajectory shape or None. Shape: {q_traj_np.shape if q_traj_np is not None else 'None'}")
                 return None

            if not np.allclose(q_traj_np[-1], q_end, atol=1e-5):
                 self.logger.warn("jtraj final step didn't exactly match q_end. Forcing it.")
                 q_traj_np[-1] = q_end

            return q_traj_np


    def setPickPose(self, x, y, z, roll, pitch, yaw):
        pose_list = [x, y, z, roll, pitch, yaw + self.angle_offset]
        self.pick_pose_se3 = self._list_to_se3(pose_list)
        if self.pick_pose_se3:
            self.logger.info(f"Pick pose set: T={self.pick_pose_se3.t}, RPY(zyx)={np.degrees(self.pick_pose_se3.rpy(order='zyx'))}")
        else:
             self.logger.error("Failed to set pick pose.")

    def setDropPose(self, x, y, z, roll, pitch, yaw):
        pose_list = [x, y, z, roll, pitch, yaw + self.angle_offset]
        self.drop_pose_se3 = self._list_to_se3(pose_list)
        if self.drop_pose_se3:
            self.logger.info(f"Drop pose set: T={self.drop_pose_se3.t}, RPY(zyx)={np.degrees(self.drop_pose_se3.rpy(order='zyx'))}")
        else:
             self.logger.error("Failed to set drop pose.")

    def setHomePose(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None):
        if all(v is None for v in [x, y, z, roll, pitch, yaw]):
            self.home_q = deepcopy(self.robot.qr)
            if len(self.home_q) < len(joint_names):
                self.home_q = np.concatenate((self.home_q, [self.gripper_open_value, self.gripper_open_value]))
            self.home_pose_se3 = self.robot.fkine(self.home_q[:self.num_robot_joints])
            self.logger.info(f"Home pose set to default joint config (qr): {self.home_q.round(3)}")
        else:
             pose_list = [x, y, z, roll, pitch, yaw + self.angle_offset]
             self.home_pose_se3 = self._list_to_se3(pose_list)
             q_home_ik_full, success = self._calculate_ik(self.home_pose_se3)
             if success:
                 self.home_q = np.concatenate((q_home_ik_full[:self.num_robot_joints], [self.gripper_open_value, self.gripper_open_value]))
                 self.logger.info(f"Home pose set to Cartesian: T={self.home_pose_se3.t}, RPY(zyx)={np.degrees(self.home_pose_se3.rpy(order='zyx'))}, q={self.home_q.round(3)}")
             else:
                 self.logger.warn("Home pose set to Cartesian, but failed to find corresponding IK. Home position might be unreachable.")
                 self.home_q = None

    def setScanPose(self, x=0.0, y=0.3, z=0.6, roll=0.0, pitch=pi, yaw=0.0):
        pose_list = [x, y, z, roll, pitch, yaw + self.angle_offset]
        self.scan_pose_se3 = self._list_to_se3(pose_list)
        if self.scan_pose_se3:
             self.logger.info(f"Scan pose set: T={self.scan_pose_se3.t}, RPY(zyx)={np.degrees(self.scan_pose_se3.rpy(order='zyx'))}")
        else:
            self.logger.error("Failed to set scan pose.")


    def setGripperPose(self, width=0.0):
        threshold = 0.001 # Use a small threshold close to zero
        if width > threshold:
            self.target_gripper_width = self.gripper_open_value
            state = "OPEN"
            self.is_gripper_open = True
        else:
            self.target_gripper_width = self.gripper_force_close_value 
            state = "CLOSED"
            self.is_gripper_open = False
        self.logger.info(f"Target gripper set to {state} (value: {self.target_gripper_width}) based on width {width}")

    def call_gripper_service(self, width):
        self.setGripperPose(width)

        q_start = deepcopy(self.current_q)
        q_end = deepcopy(q_start)
        q_end[self.gripper_indices] = self.target_gripper_width

        if np.allclose(q_start[self.gripper_indices], q_end[self.gripper_indices]):
             self.logger.info(f"Gripper already at target state (value: {self.target_gripper_width}).")
             return True

        gripper_velocity = 0.1
        gripper_traj = self._generate_trajectory(q_start, q_end, gripper_velocity, is_cartesian=False)

        if gripper_traj is None:
             self.logger.error("Failed to generate gripper trajectory.")
             return False

        # state_str = "OPEN" if self.target_gripper_width > self.gripper_closed_value else "CLOSED"
        if self.target_gripper_width > self.gripper_closed_value:
            state_str = "OPEN"
            self.is_gripper_open = True
        else:
            state_str = "CLOSED"
            self.is_gripper_open = False

        self.logger.info(f"Moving gripper to {state_str} (target value: {self.target_gripper_width})")
        success = self._execute_trajectory(gripper_traj, is_gripper_move=True)

        if success:
            #time.sleep(1.0)
            self.logger.info("Gripper move finished.")
            return True
        else:
            self.logger.error("Gripper move failed during execution.")
            return False


    def call_get_current_pose_service(self):
        if self.current_q is None:
             self.logger.error("Cannot get current pose, current_q is None.")
             return None
        try:
            current_pose_se3 = self.robot.fkine(self.current_q[:self.num_robot_joints])
            rpy = current_pose_se3.rpy(order='zyx')
            pose_list = [
                current_pose_se3.t[0], current_pose_se3.t[1], current_pose_se3.t[2],
                rpy[0], rpy[1], rpy[2]
            ]
            return pose_list
        except Exception as e:
             self.logger.error(f"Failed to calculate forward kinematics: {e}")
             return None


    def call_set_joint_velocity_service(self, velocity):
         self.logger.warn("call_set_joint_velocity_service is deprecated. Velocity is set in move functions.")
         pass

    def call_move_joint_service(self, joint_index, ang_disp, velocity=None):
         num_total_joints = len(joint_names)
         if not isinstance(joint_index, int) or joint_index < 0 or joint_index >= num_total_joints:
            self.logger.error(f"Invalid joint index {joint_index}. Must be int between 0 and {num_total_joints - 1}.")
            return False

         if velocity is None:
            velocity = self.benchmarking_velocity
            self.logger.debug(f"Using benchmarking velocity: {velocity:.2f}")

         joint_name = joint_names[joint_index]
         self.logger.info(f"Moving joint {joint_index} ({joint_name}) by {np.degrees(ang_disp):.2f} degrees (relative) using velocity {velocity:.2f} rad/s (approx).")

         q_start = deepcopy(self.current_q)

         q_end = deepcopy(q_start)
         q_end[joint_index] += ang_disp

         if joint_index < self.num_robot_joints and self.qlim is not None:
            lower_limit = self.qlim[0, joint_index]
            upper_limit = self.qlim[1, joint_index]
            target_angle = q_end[joint_index]

            if not (lower_limit <= target_angle <= upper_limit):
                self.logger.error(f"Target angle {target_angle:.4f} rad for joint {joint_index} ({joint_name}) exceeds limits [{lower_limit:.4f}, {upper_limit:.4f}]. Aborting move.")
                return False

         q_traj = self._generate_trajectory(q_start, q_end, velocity, is_cartesian=False)

         if q_traj is None:
            self.logger.error(f"Failed to generate trajectory for moving joint {joint_index} ({joint_name}).")
            return False

         success = self._execute_trajectory(q_traj)

         if success:
            self.logger.info(f"Successfully moved joint {joint_index} ({joint_name}).")
            return True
         else:
            self.logger.error(f"Failed to execute trajectory for moving joint {joint_index} ({joint_name}).")
            return False


    def move_to_pose(self, target_pose_se3: SE3, velocity: float, cartesian: bool):
        if target_pose_se3 is None:
             self.logger.error("Cannot move to a None pose.")
             return False

        if not isinstance(target_pose_se3, SE3):
            self.logger.error(f"Target pose must be an SE3 object, got {type(target_pose_se3)}")
            return False

        target_rpy_deg = np.degrees(target_pose_se3.rpy(order='zyx'))
        self.logger.info(f"Moving to pose: T={target_pose_se3.t}, RPY(zyx)={target_rpy_deg}, Cartesian={cartesian}, Vel={velocity}")

        q_start = deepcopy(self.current_q)
        try:
            T_start = self.robot.fkine(q_start[:self.num_robot_joints])
        except Exception as e:
            self.logger.error(f"FK failed for start configuration {q_start[:self.num_robot_joints]}: {e}")
            return False

        if cartesian:
             q_traj = self._generate_trajectory(q_start, None, velocity, is_cartesian=True, T_start=T_start, T_end=target_pose_se3)
             if q_traj is None:
                 self.logger.error("Cartesian trajectory generation failed (likely IK issues or ctraj failure).")
                 return False
        else:
            q_end_full, ik_success = self._calculate_ik(target_pose_se3, q_initial_guess=q_start[:self.num_robot_joints])
            if not ik_success:
                self.logger.error(f"IK failed for target pose. Cannot execute joint move.")
                return False

            q_end = np.concatenate((q_end_full[:self.num_robot_joints], q_start[self.gripper_indices]))

            q_traj = self._generate_trajectory(q_start, q_end, velocity, is_cartesian=False)
            if q_traj is None:
                 self.logger.error("Joint trajectory generation failed (jtraj failure).")
                 return False

        return self._execute_trajectory(q_traj)


    def execute_pick_up(self, use_cartesian=False):
        if self.pick_pose_se3 is None:
            self.logger.error("Pick pose not set. Cannot execute pick-up.")
            return False
        if not isinstance(self.pick_pose_se3, SE3):
             self.logger.error(f"Invalid pick_pose_se3 type: {type(self.pick_pose_se3)}")
             return False

        self.logger.info(f"--- Starting Pick Up Sequence (Cartesian Moves Preferred: {use_cartesian}) ---")

        self.logger.info("Opening gripper...")
        if not self.call_gripper_service(width=0.1):
             self.logger.error("Failed to open gripper.")
             return False

        T_pick_intermediate = deepcopy(self.pick_pose_se3)
        T_pick_intermediate.t[2] = max(self.intermediate_z_stop, T_pick_intermediate.t[2] + self.stop_above_destination)
        self.logger.info("Moving to intermediate height above pick target...")
        if not self.move_to_pose(T_pick_intermediate, self.default_velocity, cartesian=use_cartesian):
            self.logger.error("Failed to reach intermediate pick height.")
            return False

        T_pre_grasp = deepcopy(self.pick_pose_se3)
        T_pre_grasp.t[2] = self.pick_pose_se3.t[2] + self.gripper_offset + self.stop_above_destination
        self.logger.info(f"Moving to pre-grasp position (Z={T_pre_grasp.t[2]:.3f})...")
        if not self.move_to_pose(T_pre_grasp, self.approach_velocity, cartesian=use_cartesian):
            self.logger.error("Failed to reach pre-grasp pose.")
            return False

        T_grasp = deepcopy(self.pick_pose_se3)
        T_grasp.t[2] = self.pick_pose_se3.t[2] + self.gripper_offset
        self.logger.info(f"Moving down to grasp position (Z={T_grasp.t[2]:.3f})...")
        if not self.move_to_pose(T_grasp, self.approach_velocity, cartesian=True):
             self.logger.error("Failed to reach grasp pose.")
             return False

        self.logger.info("Closing gripper (with force)...")
        if not self.call_gripper_service(width=0.0):
             self.logger.warn("Gripper close command failed or took too long. Attempting retract.")

        current_pose_list = self.call_get_current_pose_service()
        if current_pose_list is None:
             self.logger.error("Failed to get current pose for retraction.")
             return False
        T_current = self._list_to_se3(current_pose_list)
        if T_current is None:
             self.logger.error("Failed to convert current pose list to SE3 for retraction.")
             return False

        T_retract_intermediate = deepcopy(T_current)
        T_retract_intermediate.t[2] = self.intermediate_z_stop
        self.logger.info(f"Retracting to intermediate height (Z={T_retract_intermediate.t[2]:.3f})...")
        if not self.move_to_pose(T_retract_intermediate, self.default_velocity, cartesian=True):
             self.logger.error("Failed to retract to intermediate height.")
             return False

        self.logger.info("--- Pick Up Sequence Finished ---")
        return True


    def execute_place(self, use_cartesian=False):
        if self.drop_pose_se3 is None:
            self.logger.error("Drop pose not set. Cannot execute place.")
            return False
        if not isinstance(self.drop_pose_se3, SE3):
             self.logger.error(f"Invalid drop_pose_se3 type: {type(self.drop_pose_se3)}")
             return False

        self.logger.info(f"--- Starting Place Sequence (Cartesian Moves Preferred: {use_cartesian}) ---")

        T_drop_intermediate = deepcopy(self.drop_pose_se3)
        T_drop_intermediate.t[2] = max(self.intermediate_z_stop, T_drop_intermediate.t[2] + self.stop_above_destination)
        self.logger.info("Moving to intermediate height above drop target...")
        if not self.move_to_pose(T_drop_intermediate, self.default_velocity, cartesian=use_cartesian):
            self.logger.error("Failed to reach intermediate drop height.")
            return False

        T_pre_release = deepcopy(self.drop_pose_se3)
        T_pre_release.t[2] = self.drop_pose_se3.t[2] + self.gripper_offset + self.stop_above_destination
        self.logger.info(f"Moving to pre-release position (Z={T_pre_release.t[2]:.3f})...")
        if not self.move_to_pose(T_pre_release, self.approach_velocity, cartesian=use_cartesian):
            self.logger.error("Failed to reach pre-release pose.")
            return False

        T_release = deepcopy(self.drop_pose_se3)
        T_release.t[2] = self.drop_pose_se3.t[2] + self.gripper_offset
        self.logger.info(f"Moving down to release position (Z={T_release.t[2]:.3f})...")
        if not self.move_to_pose(T_release, self.approach_velocity, cartesian=True):
             self.logger.error("Failed to reach release pose.")
             return False

        self.logger.info("Opening gripper...")
        if not self.call_gripper_service(width=0.1):
             self.logger.warn("Gripper open command failed or took too long. Proceeding with retract.")

        current_pose_list = self.call_get_current_pose_service()
        if current_pose_list is None:
             self.logger.error("Failed to get current pose for retraction.")
             return False
        T_current = self._list_to_se3(current_pose_list)
        if T_current is None:
             self.logger.error("Failed to convert current pose list to SE3 for retraction.")
             return False

        T_retract_intermediate = deepcopy(T_current)
        T_retract_intermediate.t[2] = self.intermediate_z_stop
        self.logger.info(f"Retracting to intermediate height (Z={T_retract_intermediate.t[2]:.3f})...")
        if not self.move_to_pose(T_retract_intermediate, self.default_velocity, cartesian=True):
             self.logger.error("Failed to retract to intermediate height after place.")
             return False

        self.logger.info("--- Place Sequence Finished ---")
        return True

    def execute_pick_and_place(self, use_cartesian=False):
        if self.execute_pick_up(use_cartesian=use_cartesian):
            self.logger.info("Pick successful, proceeding to place.")
            return self.execute_place(use_cartesian=use_cartesian)
        else:
            self.logger.error("Pick-up failed, aborting place sequence.")
            return False

    def execute_cartesian_pick_up(self):
        return self.execute_pick_up(use_cartesian=True)

    def execute_cartesian_place(self):
        return self.execute_place(use_cartesian=True)

    def execute_cartesian_pick_and_place(self):
        return self.execute_pick_and_place(use_cartesian=True)


    def reach_scanpose(self, use_cartesian=False):
        if self.scan_pose_se3 is None:
             self.logger.error("Scan pose is not set.")
             return False
        self.logger.info(f"Moving to Scan Pose (Cartesian Pref: {use_cartesian})...")
        return self.move_to_pose(self.scan_pose_se3, self.default_velocity, cartesian=use_cartesian)

    def reach_cartesian_scanpose(self):
         return self.reach_scanpose(use_cartesian=True)

    def go_home(self, use_cartesian=False):
        self.logger.info(f"Moving to Home Pose (Cartesian Pref: {use_cartesian})...")
        if self.home_q is not None and not use_cartesian:
             self.logger.info("Using joint move to home configuration.")
             q_start = deepcopy(self.current_q)
             q_traj = self._generate_trajectory(q_start, self.home_q, self.default_velocity, is_cartesian=False)
             if q_traj is None:
                  self.logger.error("Failed to generate trajectory to home joint configuration.")
                  if self.home_pose_se3 is not None:
                       self.logger.warn("Falling back to Cartesian move to home pose.")
                       return self.move_to_pose(self.home_pose_se3, self.default_velocity, cartesian=True)
                  else:
                       return False
             return self._execute_trajectory(q_traj)
        elif self.home_pose_se3 is not None:
             self.logger.info("Using Cartesian move to home pose.")
             return self.move_to_pose(self.home_pose_se3, self.default_velocity, cartesian=True)
        else:
             self.logger.error("Home pose not defined (neither joint nor Cartesian). Cannot go home.")
             return False