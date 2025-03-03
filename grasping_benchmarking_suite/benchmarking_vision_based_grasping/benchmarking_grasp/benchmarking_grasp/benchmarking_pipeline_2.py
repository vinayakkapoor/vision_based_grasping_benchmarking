#!/usr/bin/env python3
"""
Author: Vinayak Kapoor
Email: vkapoor@wpi.edu
"""
import threading
from threading import Lock
import numpy as np
import time
import enum
import os
import datetime
import csv

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import cv_bridge
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image, CameraInfo, JointState
from gazebo_msgs.srv import SpawnModel, DeleteModel
from ament_index_python.packages import get_package_share_directory


from grasp_interfaces.srv import Grasp2DPrediction, GraspPrediction
from utils import *
from pick_and_place import PickAndPlace

class BenchmarkTestStates(enum.Enum):
    """
    States to keep track of the scores for evaluation in the simulation
    Not sure how these will help duing realtime tests
    """
    FREE = 0
    PICK_UP = 1
    ROTATE = 2
    SHAKE = 3

class Benchmark(Node):
    """Perform Benchmarking

    Args:
        Node (_type_): _description_
    """
    def __init__(self):
        super().__init__('benchmarking_node')
        init_params(self)
        params = self.get_parameters_by_prefix('')
        for param_name, param in params.items():
            self.get_logger().info(f'{param_name}:{param.value}')
        
        self.use_cartesian = self.get_parameter('use_cartesian').value
        self.base_frame = self.get_parameter('base_frame').value
        self.sim_mode = self.get_parameter('sim_mode').value
        if self.sim_mode:
            self.camera_frame = self.get_parameter('camera_frame_sim').value
            self.rgb_image_topic = self.get_parameter('rgb_image_sim').value
            self.depth_image_topic = self.get_parameter('depth_image_sim').value
            self.cam_info_topic = self.get_parameter('cam_info_sim').value
            
        else:
            self.camera_frame = self.get_parameter('camera_frame').value
            self.rgb_image_topic = self.get_parameter('rgb_image').value
            self.depth_image_topic = self.get_parameter('depth_image').value
            self.cam_info_topic = self.get_parameter('cam_info_depth').value        
        self.visualisation_topic = self.get_parameter('visualisation').value      
        self.grasp_in_image_frame_service = self.get_parameter('grasp_in_image_frame').value
        self.vis_type = self.get_parameter('visualisation_type').value 
        self.gripper_width = self.get_parameter('gripper_width').value 
        self.gripper_height = self.get_parameter('gripper_height').value 
        self.gripper_offset = self.get_parameter('gripper_offset').value 
        self.intermediate_z_stop = self.get_parameter('intermediate_z_stop').value
        self.bad_grasp_z = self.get_parameter('bad_grasp_z').value
        self.enable_benchmark_test = self.get_parameter('enable_benchmark_test').value
        self.benchmarking_velocity = self.get_parameter('benchmarking_velocity').value
        self.robot_default_velocity = self.get_parameter('robot_default_velocity').value
        self.scan_pose = self.get_parameter('scan_pose').value
        self.urdf_package_name = self.get_parameter('urdf_package_name').value
        self.experiments_package_name = self.get_parameter('experiments_package_name').value
        self.experiments_config_relative_path = self.get_parameter('experiments_config_relative_path').value
        self.x_offset = self.get_parameter('x_offset_world').value
        self.y_offset = self.get_parameter('y_offset_world').value



        self.lock = Lock()
        self.reentrant_callback_group = ReentrantCallbackGroup()
        self.bridge = cv_bridge.CvBridge()

        # Initialise services
        self.grasp_client = self.create_client(
            Grasp2DPrediction,
            self.grasp_in_image_frame_service,
            callback_group=self.reentrant_callback_group
        )
        # NOT TESTED
        self.spawn_model_client = self.create_client(
            SpawnModel,
            '/gazebo/spawn_sdf_model',
            callback_group=self.reentrant_callback_group
        )
        # NOT TESTED
        self.delete_model_client = self.create_client(
            DeleteModel,
            '/gazebo/delete_model',
            callback_group=self.reentrant_callback_group
        )

        # Initialise topics
        self.rgb_img_sub = self.create_subscription(
            Image,
            self.rgb_image_topic,
            self._rgb_img_callback, 1,
            callback_group=self.reentrant_callback_group
        )
        self.depth_img_sub = self.create_subscription(
            Image,
            self.depth_image_topic,
            self._depth_img_callback, 1,
            callback_group=self.reentrant_callback_group
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            self.cam_info_topic,
            self._cam_info_callback, 1,
            callback_group=self.reentrant_callback_group            
        )
        self.joint_states_sub = self.create_subscription( # NOT TESTED
            JointState,
            '/joint_states',
            self._joint_states_cb, 1,
            callback_group=self.reentrant_callback_group
        )
        # ALTERNATIVE to GazeboGraspEvent??
        # self.grasp_sub = self.create_subscription(
        #     GazeboGraspEvent,
        #     '/gazebo_grasp_plugin_event_republisher/grasp_events',
        #     self._on_grasp_event, 1,
        #     callback_group=self.reentrant_callback_group
        # )

        self.img_vis_pub = self.create_publisher(
            Image,
            self.visualisation_topic,
            10,  
            callback_group=self.reentrant_callback_group
        )
        

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_rgb_image = None
        self.current_depth_image = None
        self.cam_info:CameraInfo = None
        self.cam_K = 0    

        self.depth_scale = 1   
        self.pick_and_place = PickAndPlace(self, 
                                           gripper_offset=self.gripper_offset,
                                           intermediate_z_stop=self.intermediate_z_stop)
        
        self.urdf_path = os.path.join(get_package_share_directory(self.urdf_package_name), "urdf/objects")
        self.experiments_config_path = os.path.join(get_package_share_directory(self.experiments_package_name), self.experiments_config_relative_path)
        PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
        self.RESULTS_DIR = os.path.join(PACKAGE_ROOT, "src/benchmarking_grasp/results")
        os.makedirs(self.RESULTS_DIR, exist_ok=True)
        start_time_str = str(datetime.datetime.now().strftime("%I:%M%p on %B %d, %Y"))
        self.RESULTS_FILE = os.path.join(self.RESULTS_DIR, "benchmarking_result_" +  start_time_str + ".csv")
        print(self.RESULTS_FILE, "----------------------------")
        with open(self.RESULTS_FILE, 'w') as file:
            header = ['Experiment', 'Trial', 'Object', 'Pose', 'Score', 'Inference Time']
            writer = csv.writer(file)
            writer.writerow(header)

        # Parse object list and benchmarking parameters from the experiments config file
        parsed_experiments = parse_experiments_config(self.experiments_config_path, self.urdf_path)
        # Generate object list and benchmarking parameters for spawning objects
        self.experiments = generate_experiments(parsed_experiments)

        # Variables to track 
        self.experiment_idx = 0
        self.pose_idx = 0
        self.object_idx = 0
        self.n_ = 0
        self.testing_in_process = False
        self.benchmark_state = BenchmarkTestStates.FREE
        self.attached = False
        self.positive_grasps = []
        self.negative_grasps = []
        self.finger1_state = 0.05
        self.finger2_state = 0.05
        self.avg_inference_time = 0

        rclpy.get_default_context().on_shutdown(self.on_shutdown)


    def _rgb_img_callback(self, msg:Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        with self.lock:
            self.current_rgb_image = img

    def _depth_img_callback(self, msg:Image):
        img = self.bridge.imgmsg_to_cv2(msg)
        with self.lock:
            self.current_depth_image = img
    
    def _cam_info_callback(self, msg:CameraInfo):
        if self.cam_info is None: # cam_info is essentially a flag, to prevent locking on every callback
            with self.lock:        
                self.cam_info = msg
                self.cam_K = np.array(msg.k).reshape((3, 3))
    
    def _joint_states_cb(self, msg:JointState):
        """
        Callback for joint states
        """
        position = msg.position
        with self.lock:
            self.finger1_state = position[0]
            self.finger2_state = position[1]

    # NOT TESTED
    def _on_grasp_event(self, msg):
        """
        If a grasp is detected by the grasp plugin
        Do stuff
        """
        object = msg.object
        attached = msg.attached

        if attached:
            with self.lock:
                self.pick_and_place.call_gripper_service(self.finger1_state)
                        
        if attached and self.testing_in_process:
            self.attached = True
        
        if not attached and self.testing_in_process:
            self.negative_grasps.append(object)
            self.attached = False
        
        if not attached and not self.testing_in_process and self.attached:
            self.positive_grasps.append(object)
            self.attached = False

    def on_shutdown(self):
        self.get_logger().info(f"\033[92m Average inference time: {self.avg_inference_time} secs\033[0m")
                
    def get_grasp_in_img_frame(self):
        grasp_request:Grasp2DPrediction.Request = Grasp2DPrediction.Request()

        rgb_img = None
        depth_img = None
        with self.lock:
            while self.current_depth_image is None or self.current_rgb_image is None:
                self.get_logger().warning('Waiting for rgb/depth image')
                time.sleep(0.1) # Since the callback groups are reentrant, this should not hold up the thread 
            
            rgb_img = self.current_rgb_image.copy()
            depth_img = self.current_depth_image.copy()


        grasp_request.rgb_image = self.bridge.cv2_to_imgmsg(rgb_img, encoding='rgb8')
        grasp_request.depth_image = self.bridge.cv2_to_imgmsg(depth_img)

        if not self.grasp_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Grasp service is not available!")
            raise RuntimeError("Grasp service is not available!")            

        future = self.grasp_client.call_async(grasp_request)
        self.get_logger().info('Waiting for the service to return')
        rclpy.spin_until_future_complete(self, future=future)
        
        result = future.result()
        if result is None:
            self.get_logger().warning("Grasp Service failed to return a valid result")
            return None, None
        
        return result, rgb_img, depth_img
    
    def start_complete_benchmarking(self):
        """
        Executes the entire benchmarking procedure
        1. Spawns an object in Gazebo / waits for user to place the object as requested
        2. Calls predict service to get the 3D coordinates of the grasp
        3. Picks up the object
        4. Performs the benchmarking maneuvers
        5. Places the object (tracks the state all the way)
        6. Updates the score log file (Only useful for simulator)
        7. Deletes the object from environment (Only useful for simulator)
        """
        skip = False
        experiment = self.experiments[self.experiment_idx]
        object = experiment[0][self.object_idx]
        pose = experiment[1][self.pose_idx]
        # rospy.set_param("current_recording", str(object.split("/")[-1].split(".")[0]) + "_" + str(self.pose_idx))

        if self.sim_mode:
            self.spawn_model(object, pose)
            self.get_logger().info("Spawning {} at pose {}".format(str(object.split("/")[-1].split(".")[0]), self.pose_idx))
            time.sleep(2)
        else:
            try:
                input("\033[92mPlace {} at pose {} and press 'enter'".format(str(object.split("/")[-1].split(".")[0]), self.pose_idx)+"\033[0m")
            except SyntaxError:
                pass

        # Execute the benchmark test
        self.testing_in_process = True
        # rospy.set_param("start_recording", True)
        
        try:
            success = self.execute_pickup()
            if self.enable_benchmark_test:
                self.execute_benchmarking_maneuvers()
            score = self.benchmark_state
            if success:
                self.execute_place()
        except Exception as e:
            self.get_logger().error(f"Skipping this turn {e}")
            skip = True

        self.testing_in_process = False

        if self.use_cartesian:
            self.pick_and_place.reach_cartesian_scanpose()
        else:
            self.pick_and_place.reach_scanpose()

        if self.sim_mode:
            try:
                time.sleep(1)
                self.delete_model(object)
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f"Object deleted while still attached to hand {e}")

        if not skip:
            with open(self.log_file_path, 'a') as file:
                update = [str(self.experiment_idx), str(self.n_), str(object.split("/")[-1].split(".")[0]), str(self.pose_idx), score.value, str(self.inference_time)]
                writer = csv.writer(file)
                writer.writerow(update)

            # Track the status of the test
            self.pose_idx = self.pose_idx + 1 
            if self.pose_idx >= len(experiment[1]):
                self.pose_idx = 0
                self.n_ = self.n_ + 1
                if self.n_ >= experiment[2]:
                    self.n_ = 0
                    self.object_idx = self.object_idx + 1
                    if self.object_idx >= len(experiment[0]):
                        self.object_idx = 0
                        self.experiment_idx = self.experiment_idx + 1
                        # rospy.loginfo("[Benchmarking Pipeline] Success rate for experiment %s: %s", self.experiment_idx, len(self.positive_grasps)/(len(self.positive_grasps) + len(self.negative_grasps)))
                        if self.experiment_idx >= len(self.experiments):
                            self.get_logger().info("Benchmarking test completed successfully")
                            self.destroy_node()
        
        # rospy.set_param("start_recording", False)

    # NOT TESTED
    def execute_pickup(self):
        """
        1. Gets the world frame coordinates for grasp using the grasping algorithm
        2. Picks up the object from the received coordinate
        3. Benchmarking state is updated to PICK_UP
        """
        start_time = self.get_clock().now()
        
        grasp_in_img_frame, rgb_img, depth_img = self.get_grasp_in_img_frame()
        grasp_in_cam_frame = self.grasp_img2cam(grasp_in_img_frame, rgb_img, depth_img)
        grasp_in_world_frame:GraspPrediction.Response = self.grasp_cam2world(grasp_in_cam_frame)
        
        end_time = self.get_clock().now()
    

        # Calculate average inference time
        self.inference_time = (end_time - start_time).nanoseconds/10e-9
        total_objects = len(self.experiments[self.experiment_idx][0])
        total_poses = len(self.experiments[self.experiment_idx][1])
        total_samples = self.n_*total_objects*total_poses + self.object_idx*total_poses + self.pose_idx + 1
        self.avg_inference_time =  (self.avg_inference_time * (total_samples - 1) + self.inference_time)/total_samples 

        
        # Extracting grasp coordinates
        x = grasp_in_world_frame.best_grasp.pose.position.x + self.x_offset
        y = grasp_in_world_frame.best_grasp.pose.position.y + self.y_offset
        z = grasp_in_world_frame.best_grasp.pose.position.z
        (rx, ry, rz) = euler_from_quaternion([grasp_in_world_frame.best_grasp.pose.orientation.x, grasp_in_world_frame.best_grasp.pose.orientation.y, 
                                              grasp_in_world_frame.best_grasp.pose.orientation.z, grasp_in_world_frame.best_grasp.pose.orientation.w])

        if z > self.bad_grasp_z:
            yaw = (rz + np.pi/2) % np.pi - np.pi/2  # Wrap [-np.pi/2, np.pi/2]
            self.pick_and_place.setPickPose(x=x, y=y, z=z, roll=np.pi, pitch=0, yaw=yaw)
            self.pick_and_place.setDropPose(x=x, y=y, z=z, roll=np.pi, pitch=0, yaw=yaw)
            self.pick_and_place.setGripperPose(width=0.00)

            if self.use_cartesian:
                self.pick_and_place.execute_cartesian_pick_up()
            else:
                self.pick_and_place.execute_pick_up()
            if self.attached:
                self.benchmark_state = BenchmarkTestStates.PICK_UP
        else:
            self.get_logger().warning("Bad grasp received, skipping this turn")
            return False

        return True
    
    # NOT TESTED
    def execute_place(self):
        """
        1. Places the object 
        2. Benchmarking state is updated to FREE
        """
        self.benchmark_state = BenchmarkTestStates.FREE
        
        if self.use_cartesian:
            self.pick_and_place.execute_cartesian_place()
        else:
            self.pick_and_place.execute_place()
    
    # NOT TESTED
    def execute_benchmarking_maneuvers(self):
        """
        1. Performs benchmarking maneuvers
        2. Updates state as applicable
        3. Two maneuvers
            1. Roll pitch yaw rotation
            2. Shake test
        """
        self.pick_and_place.call_set_joint_velocity_service(self.benchmarking_velocity)

        # Rotate the object
        pose = self.pick_and_place.call_get_current_pose_service()
        (x, y, z) = (pose.position.x, pose.position.y, pose.position.z) 
        (roll, pitch, yaw) = euler_from_quaternion((pose.orientation.x, pose.orientation.y, 
                                                                       pose.orientation.z, pose.orientation.w))
        
        # Yawing    
        self.pick_and_place.call_move_joint_service(6, np.pi/4)
        self.pick_and_place.call_move_joint_service(6, -np.pi/2)
        self.pick_and_place.call_move_joint_service(6, np.pi/4)

        if self.attached:
            self.benchmark_state = BenchmarkTestStates.ROTATE

        # Rolling
        self.pick_and_place.call_move_joint_service(4, np.pi/8)
        self.pick_and_place.call_move_joint_service(4, -np.pi/4)
        self.pick_and_place.call_move_joint_service(4, np.pi/4)
        self.pick_and_place.call_move_joint_service(4, -np.pi/4)
        self.pick_and_place.call_move_joint_service(4, np.pi/8)

        if self.attached:
            self.benchmark_state = BenchmarkTestStates.SHAKE

        self.pick_and_place.call_set_joint_velocity_service(self.robot_default_velocity)
    
    
    def test_execute_benchmarking(self):
        self.get_logger().info("Executing benchmark")
        if not self.over_head:
            self.pick_and_place.setScanPose(x=self.scan_pose[0], y=self.scan_pose[1], z=self.scan_pose[2], 
                                            roll=self.scan_pose[3], pitch=self.scan_pose[4], yaw=self.scan_pose[5])
            if self.use_cartesian:
                self.pick_and_place.reach_cartesian_scanpose()
            else:
                self.pick_and_place.reach_scanpose()
                
        for i in range(10):
            time.sleep(5)
            grasp_in_img_frame, rgb_img, depth_img = self.get_grasp_in_img_frame()
            grasp_in_cam_frame = self.grasp_img2cam(grasp_in_img_frame, rgb_img, depth_img)
            grasp_in_world_frame = self.grasp_cam2world(grasp_in_cam_frame)

            print(grasp_in_world_frame)
            # self.get_logger().info(self.get_grasp_in_img_frame())


    def grasp_img2cam(self, grasp_in_img_frame:Grasp2DPrediction.Response, rgb_img, depth_img):
        center = [grasp_in_img_frame.best_grasp.px, grasp_in_img_frame.best_grasp.py]
        precrop_center = center.copy() 
        angle = grasp_in_img_frame.best_grasp.angle

        angle_vec_in_cam = np.linalg.inv(self.cam_K)@np.array([[np.cos(angle)], [np.sin(angle)], [1]])
        origin_vec_in_cam = np.linalg.inv(self.cam_K)@np.array([[0], [0], [1]])
        angle_in_cam = np.arctan2(angle_vec_in_cam[1, 0] - origin_vec_in_cam[1, 0], angle_vec_in_cam[0, 0] - origin_vec_in_cam[0, 0])
        self.angle_2d_in_cam_frame = angle_in_cam

        (position, angle_in_cam, gripper_width) = calculate_camera_frame_transform(
            center=center,
            precrop_center=precrop_center,
            angle=angle,
            depth_image=depth_img,
            cam_K=self.cam_K,
            depth_scale=self.depth_scale,
            angle_2d_flag=self.angle_2d_in_cam_frame,
            gripper_width=self.gripper_width,
            gripper_height=self.gripper_height
        )

        result = GraspPrediction.Response()
        result.success = True
        result.best_grasp.pose.position.x = position[0]
        result.best_grasp.pose.position.y = position[1]
        result.best_grasp.pose.position.z = position[2]
        result.best_grasp.pose.orientation = list_to_quaternion(quaternion_from_euler(0,0,angle_in_cam))
        result.best_grasp.width = grasp_in_img_frame.best_grasp.width
        result.best_grasp.quality = grasp_in_img_frame.best_grasp.quality

        # visualisation
        width = result.best_grasp.width
        if self.vis_type == 'depth':            
            vis_img = draw_angled_rect(normalize_depth(depth_img), precrop_center[1], precrop_center[0], angle, width=width, height=width/2)
        else:
            vis_img = draw_angled_rect(rgb_img, precrop_center[1], precrop_center[0], angle, width=width, height=width/2)
        msg = self.bridge.cv2_to_imgmsg(vis_img, encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.img_vis_pub.publish(msg)
        
        return result
    
    def spawn_model(self, model_path, pose):
        """
        Spawns model in the required pose in Gazebo
        """
        if not self.spawn_model_client.service_is_ready():
            self.get_logger().warn('Spawn model service is not available')
            return False
        
        spawn_pose = Pose()
        spawn_pose.position.x = pose[0]
        spawn_pose.position.y = pose[1]
        spawn_pose.position.z = pose[2]

        quaternion = quaternion_from_euler (pose[3], pose[4], pose[5])
        spawn_pose.orientation.x = quaternion[0]
        spawn_pose.orientation.y = quaternion[1]
        spawn_pose.orientation.z = quaternion[2]
        spawn_pose.orientation.w = quaternion[3]

        file_xml = open(model_path, 'r')
        model_xml = file_xml.read()
        
        request = SpawnModel.Request()
        request.initial_pose = spawn_pose
        request.model_xml = model_xml
        request.model_name = model_path.split("/")[-1].split(".")[0]
        request.reference_frame = 'world'
        request.robot_namespace = ''
        
        future = self.spawn_model_client.call_async(request)
        rclpy.spin_until_future_complete(future)
        result:SpawnModel.Response = future.result()
        return result.success
    
    # NOT TESTED
    def delete_model(self, model_path):
        """
        Deletes the model from the Gazebo environment
        """        
        if not self.delete_model_client.service_is_ready():
            self.get_logger().warn('Delete model service is not available')
            return False
        request = DeleteModel.Request()
        request.model_name = model_path.split("/")[-1].split(".")[0]
        future = self.delete_model_client.call_async(request)
        rclpy.spin_until_future_complete(future)
        result:DeleteModel.Response = future.result()
        return result.success
    
    def grasp_cam2world(self, grasp_in_cam_frame:GraspPrediction.Response):
        result = GraspPrediction.Response()
        pose_in_world = self.convert_pose(grasp_in_cam_frame.best_grasp.pose, self.camera_frame, self.base_frame)
        if pose_in_world is None:
            result.success = False
            return result
        
        result.success = True
        result.best_grasp.pose = pose_in_world

        self.get_logger().info(
            f"[Cam To World] Grasp in world frame: {pose_in_world.position.x}, "
            f"{pose_in_world.position.y}, {pose_in_world.position.z}"
        )

        return result
    
    def convert_pose(self, pose:Pose, from_frame, to_frame):
        """
        Convert a pose between frames using tf2.

        :param pose: geometry_msgs.msg.Pose, the pose in the original frame.
        :param from_frame: str, the original frame of the pose.
        :param to_frame: str, the target frame to transform the pose into.
        :return: geometry_msgs.msg.Pose, the transformed pose.
        """

        try:
            # Lookup the transform from 'from_frame' to 'to_frame'.
            # Passing a zero time gets the latest available transform.
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(
                f"FAILED TO GET TRANSFORM from {from_frame} to {to_frame}: {e}"
            )
            return None
        
        spose = PoseStamped()
        spose.pose = pose
        spose.header.stamp = self.get_clock().now().to_msg()
        spose.header.frame_id = from_frame

        try:
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(spose, transform)
        except Exception as e:
            self.get_logger().error(f"Error transforming pose: {e}")
            return None

        return transformed_pose.pose



def run_benchmark_in_thread(benchmark_node:Benchmark):
    benchmark_node.start_complete_benchmarking()

def main(args=None):
    rclpy.init(args=args)
    benchmark_node = Benchmark()

    try:
        # Start benchmarking in a separate thread
        benchmark_thread = threading.Thread(target=run_benchmark_in_thread, args=(benchmark_node,))
        benchmark_thread.start()

        rclpy.spin(benchmark_node, executor=MultiThreadedExecutor())
        benchmark_thread.join()

    except KeyboardInterrupt:
        rclpy.logging.get_logger().warning("Benchmarking Stopped")
    finally:
        benchmark_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



