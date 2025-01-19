#!/usr/bin/env python

from __future__ import division, print_function

import sys
ROOT = '../'  # root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

import rospy
import time
import math
import numpy as np
import cv2
from tf import transformations as tft
import matplotlib

from grasp import run_detector
from ggcnn.srv import GraspPrediction, GraspPredictionResponse
from sensor_msgs.msg import Image, CameraInfo
import geometry_msgs.msg as gmsg
import tf2_ros
import tf2_geometry_msgs
from tf import TransformerROS

import cv_bridge
bridge = cv_bridge.CvBridge()
tfBuffer = None
listener = None

class GraspService:
    def __init__(self):
        # Get the camera parameters
        # cam_info_topic = '/kinect/rgb/camera_info' #org
        cam_info_topic = '/panda_camera/rgb/camera_info'
        rospy.loginfo("waiting for camera topic: %s", cam_info_topic)
        camera_info_msg = rospy.wait_for_message(cam_info_topic, CameraInfo)
        # To manually enter the camera matrix
        # K = [886.8075059058992, 0.0, 512.5, 0.0, 886.8075059058992, 512.5, 0.0, 0.0, 1.0]
        # self.cam_K = np.array(K).reshape((3, 3))
        
        self.cam_K = np.array(camera_info_msg.K).reshape((3, 3))
        rospy.loginfo("Camera matrix extraction successful")
        
        self.img_pub = rospy.Publisher('~visualisation', Image, queue_size=1)
        rospy.Service('~predict', GraspPrediction, self.compute_service_handler) #org
        # rospy.Service('/deep_predict', GraspPrediction, self.compute_service_handler)
        
        self.base_frame = 'panda_link0'
        self.camera_frame = 'camera_link'
        self.cam_fov = 65.5
        
        self.curr_depth_img = None
        self.curr_rgb_image = None
        self.curr_img_time = 0
        self.last_image_pose = None
        # rospy.Subscriber('/kinect/depth/image_raw', Image, self._depth_img_callback, queue_size=1) #org
        rospy.Subscriber('/panda_camera/depth/image_raw', Image, self._depth_img_callback, queue_size=1)

        self.waiting = False
        self.received = False
    
    def _depth_img_callback(self, msg):
        # Doing a rospy.wait_for_message is super slow, compared to just subscribing and keeping the newest one.
        if not self.waiting:
          return
        self.curr_img_time = time.time()
        self.last_image_pose = self.current_robot_pose(self.base_frame, self.camera_frame)
        self.curr_depth_img = bridge.imgmsg_to_cv2(msg)
        # rgb = rospy.wait_for_message("/kinect/rgb/image_raw", Image) #org
        rgb = rospy.wait_for_message("/panda_camera/rgb/image_raw", Image)
        rgb = bridge.imgmsg_to_cv2(rgb, rgb.encoding)
        self.curr_rgb_image = rgb

        self.received = True

    def compute_service_handler(self, req):
        # if self.curr_depth_img is None:
        #     rospy.logerr('No depth image received yet.')
        #     rospy.sleep(0.5)

        # if time.time() - self.curr_img_time > 0.5:
        #     rospy.logerr('The Realsense node has died')
        #     return GraspPredictionResponse()

        self.waiting = True
        while not self.received:
          rospy.sleep(0.01)
        self.waiting = False
        self.received = False

        depth = self.curr_depth_img.copy()
        rgb = self.curr_rgb_image.copy()
        rgb[:, :, 2] = depth

        camera_pose = self.last_image_pose
        cam_p = camera_pose.position

        camera_rot = tft.quaternion_matrix(self.quaternion_to_list(camera_pose.orientation))[0:3, 0:3]

        rospy.loginfo("checking for grasps")
        # Do grasp prediction
        bounding_box, angle = run_detector(rgb)
        print(bounding_box)
        print("angle", angle)
        center = ((bounding_box[0] + bounding_box[2])/2, (bounding_box[1] + bounding_box[3])/2)  
        center = [bounding_box[1], bounding_box[0]]
        # center = bounding_box
        # angle = -angle      
        rospy.loginfo("grasp found")
        # Convert from image frame to camera frame

        x = (center[0] - self.cam_K[0, 2])/self.cam_K[0, 0]
        y = (center[1] - self.cam_K[1, 2])/self.cam_K[1, 1]
        z = depth[int(center[0])][int(center[1])]

        # angle -= camera_rot[0, 1]  # Correct for the rotation of the camera
        angle = (angle + np.pi/2) % np.pi - np.pi/2  # Wrap [-np.pi/2, np.pi/2]
                
        # Convert from camera frame to world frame 
        pos = np.dot(camera_rot, np.stack((x, y, z))).T + np.array([[cam_p.x, cam_p.y, cam_p.z]])
        # print(pos)

        ret = GraspPredictionResponse()
        ret.success = True
        g = ret.best_grasp
        g.pose.position.x = pos[0][0]
        g.pose.position.y = pos[0][1]
        g.pose.position.z = pos[0][2]
        g.pose.orientation = self.list_to_quaternion(tft.quaternion_from_euler(np.pi, 0, (angle - np.pi/2)))
        g.width = 1
        g.quality = 1

        self.draw_angled_rect(rgb, center[0], center[1], angle)

        return ret

    def draw_angled_rect(self, image, x, y, angle, width = 180, height = 100):
        print(x, y, angle, image.shape)
        _angle = -angle
        b = math.cos(_angle) * 0.5
        a = math.sin(_angle) * 0.5

        gray_image = image.copy()
        display_image = cv2.applyColorMap((gray_image * 255).astype(np.uint8), cv2.COLORMAP_BONE)

        pt0 = (int(x - a * height - b * width), int(y + b * height - a * width))
        pt1 = (int(x + a * height - b * width), int(y - b * height - a * width))
        pt2 = (int(2 * x - pt0[0]), int(2 * y - pt0[1]))
        pt3 = (int(2 * x - pt1[0]), int(2 * y - pt1[1]))

        cv2.line(display_image, pt0, pt1, (255, 0, 0), 5)
        cv2.line(display_image, pt1, pt2, (0, 0, 0), 5)
        cv2.line(display_image, pt2, pt3, (255, 0, 0), 5)
        cv2.line(display_image, pt3, pt0, (0, 0, 0), 5)
        cv2.circle(display_image, ((pt0[0] + pt2[0])//2, (pt0[1] + pt2[1])//2), 2, (255, 255, 0), -1)

        self.img_pub.publish(bridge.cv2_to_imgmsg(display_image, encoding="rgb8"))


    def list_to_quaternion(self, l):
        q = gmsg.Quaternion()
        q.x = l[0]
        q.y = l[1]
        q.z = l[2]
        q.w = l[3]
        return q

    def quaternion_to_list(self, quaternion):
        return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

    def current_robot_pose(self, reference_frame, base_frame):
        """
        Get the current pose of the robot in the given reference frame
            reference_frame         -> A string that defines the reference_frame that the robots current pose will be defined in
        """
        # Create Pose
        p = gmsg.Pose()
        p.orientation.w = 1.0

        # Transforms robots current pose to the base reference frame
        return self.convert_pose(p, base_frame, reference_frame)
      
    def _init_tf(self):
        # Create buffer and listener
        # Something has changed in tf that means this must happen after init_node
        global tfBuffer, listener
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
    
    def convert_pose(self, pose, from_frame, to_frame):
        """
        Convert a pose or transform between frames using tf.
            pose            -> A geometry_msgs.msg/Pose that defines the robots position and orientation in a reference_frame
            from_frame      -> A string that defines the original reference_frame of the robot
            to_frame        -> A string that defines the desired reference_frame of the robot to convert to
        """
        global tfBuffer, listener

        if tfBuffer is None or listener is None:
            self._init_tf()

        try:
            trans = tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
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


if __name__ == "__main__":
    rospy.init_node('resnet_deep_grasp')
    grasp_Service = GraspService()
    rospy.spin()
