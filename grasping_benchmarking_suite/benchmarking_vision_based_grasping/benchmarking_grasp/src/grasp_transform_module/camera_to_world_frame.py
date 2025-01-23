'''
Author: Bharath Kumar 
Email: kumar7bharath@gmail.com
'''

import rospy
import tf2_ros
import numpy as np
from tf import transformations as tft

import tf2_geometry_msgs
import geometry_msgs.msg as gmsg
from benchmarking_msgs.srv import GraspPrediction, GraspPredictionResponse

class CameraToWorldFrame:
    '''
    Converts coordinates from camera frame to the world frame based on 
    Franka Panda's robot description
    '''
    def __init__(self, sim_mode=True):
        self.sim_mode = sim_mode
        self.grasp_in_world_frame_topic = rospy.get_param("grasp_in_world_frame")
        self.grasp_in_camera_frame_topic = rospy.get_param("grasp_in_camera_frame")

        # For TF related calculations
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Frame names for calculation transform between them
        self.base_frame = rospy.get_param("base_frame")
        if self.sim_mode:
            self.camera_frame = rospy.get_param("camera_frame_sim")
        else:
            self.camera_frame = rospy.get_param("camera_frame")        

        rospy.Service(self.grasp_in_world_frame_topic, GraspPrediction, self.transform_coords_cb)
        rospy.loginfo("[Camera to World] Node loaded successfully")

    def transform_coords_cb(self, req):
        '''
        Service callback that transforms the coordinates and returns the resulting pose
        '''
        pose_in_cam = self.get_grasp_coords_in_cam_frame()

        # Transform position
        pose_in_world = self.convert_pose(pose_in_cam.pose, self.camera_frame, self.base_frame)

        # Response message
        ret = GraspPredictionResponse()
        ret.success = True
        
        g = ret.best_grasp
        g.pose = pose_in_world
        
        # TODO consider these values later 
        g.width = 0
        g.quality = 0

        rospy.loginfo("[Cam To World] Grasp in world frame: %s, %s, %s", g.pose.position.x, g.pose.position.y, g.pose.position.z)

        return ret

    def get_grasp_coords_in_cam_frame(self):
        '''
        Calls a service that obtains the coordinates of detected grasp in the camera frame
        '''
        rospy.wait_for_service(self.grasp_in_camera_frame_topic, timeout=30)

        try:
            srv_handle = rospy.ServiceProxy(self.grasp_in_camera_frame_topic, GraspPrediction)
            srv_resp = srv_handle()
            
            return srv_resp.best_grasp
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)


    def convert_pose(self, pose, from_frame, to_frame):
        """
        Convert a pose or transform between frames using tf.
            pose - A geometry_msgs.msg/Pose that defines the robots position and orientation in a reference_frame
            from_frame - A string that defines the original reference_frame of the robot
            to_frame - A string that defines the desired reference_frame of the robot to convert to
        """

        if self.tfBuffer is None or self.listener is None:
            self._init_tf()

        try:
            trans = self.tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
            return None

        spose = gmsg.PoseStamped()
        spose.pose = pose
        spose.header.stamp = rospy.Time().now
        spose.header.frame_id = from_frame

        p2 = tf2_geometry_msgs.do_transform_pose(spose, trans)

        return p2.pose

    def list_to_quaternion(self, l):
        q = gmsg.Quaternion()
        q.x = l[0]
        q.y = l[1]
        q.z = l[2]
        q.w = l[3]
        return q
