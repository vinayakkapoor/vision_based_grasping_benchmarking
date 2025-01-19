'''
Author: Bharath Kumar 
Email: kumar7bharath@gmail.com
'''

import rospy
import cv_bridge
import numpy as np
import cv2
from tf import transformations as tft

import geometry_msgs.msg as gmsg
from sensor_msgs.msg import Image, CameraInfo
from benchmarking_msgs.srv import GraspPrediction, GraspPredictionResponse, Grasp2DPrediction, \
                    Grasp2DPredictionRequest  

class ImageToCameraFrame:
    '''
    Transforms the detected grasp in 2D depth image frame to 3D coordinates in the camera frame
    '''
    def __init__(self, sim_mode=True, crop=True):
        self.sim_mode = sim_mode
        self.crop = crop
        self.use_depth_completion = rospy.get_param("use_depth_completion") 
        self.align_depth = rospy.get_param("align_depth") 
        self.crop_by_pixel = rospy.get_param("crop_by_pixel") 

        # Get topic names
        self.grasp_in_camera_frame_topic = rospy.get_param("grasp_in_camera_frame")
        self.grasp_in_image_frame_topic = rospy.get_param("grasp_in_image_frame")
        self.depth_complete_image_topic = rospy.get_param("depth_complete_image")
        self.depth_image_topic = rospy.get_param("depth_image")
        self.depth_wo_align_image = rospy.get_param("depth_wo_align_image")
        self.rgb_image_topic = rospy.get_param("rgb_image")
        self.depth_image_sim_topic = rospy.get_param("depth_image_sim")
        self.rgb_image_sim_topic = rospy.get_param("rgb_image_sim")
        self.remove_noisy_ground_plane = rospy.get_param("remove_noisy_ground_plane")
        self.angle_2d_in_cam_frame = rospy.get_param("angle_2d_in_cam_frame")
        
        self.gripper_height = rospy.get_param("gripper_height")
        self.gripper_width = rospy.get_param("gripper_width")

        self.bridge = cv_bridge.CvBridge()

        self.curr_depth_img = None
        self.curr_rgb_img = None
        self.depth_scale = 1

        # Variables makes sure of the following 
        # 1. Transform service on call waits until the depth and rgb images are received
        # 2. The depth and the rgb image are only updated when the service is called
        self.waiting = False
        self.received = False
        self.rgb_received = False

        # Indies 0, 2: v and 1, 3: u
        self.crop_size = rospy.get_param("depth_crop_size") 

        # Get camera info and subscribe to rgb and depth images
        if self.sim_mode:
            cam_info_topic = rospy.get_param("cam_info_sim")
            rospy.Subscriber(self.depth_image_sim_topic, Image, self._depth_img_callback, queue_size=1)
            rospy.Subscriber(self.rgb_image_sim_topic, Image, self._rgb_img_callback, queue_size=1)
        else:
            self.depth_scale = 0.001  # Depth scale of realsense
            if self.align_depth:
                cam_info_topic = rospy.get_param("cam_info_depth_align")
            else:
                cam_info_topic = rospy.get_param("cam_info_depth")

            if self.use_depth_completion:
                rospy.Subscriber(self.depth_complete_image_topic, Image, self._depth_img_callback, queue_size=1)
            else:
                if self.align_depth:
                    rospy.Subscriber(self.depth_image_topic, Image, self._depth_img_callback, queue_size=1)
                else:
                    rospy.Subscriber(self.depth_wo_align_image, Image, self._depth_img_callback, queue_size=1)
    
            rospy.Subscriber(self.rgb_image_topic, Image, self._rgb_img_callback, queue_size=1)

        # To manually enter the camera matrix
        # K = [886.8075059058992, 0.0, 512.5, 0.0, 886.8075059058992, 512.5, 0.0, 0.0, 1.0]
        # self.cam_K = np.array(K).reshape((3, 3))

        # Get camera matrix from info topic        
        self.camera_info_msg = rospy.wait_for_message(cam_info_topic, CameraInfo)
        self.cam_K = np.array(self.camera_info_msg.K).reshape((3, 3))
        self.depth_image_size = [self.camera_info_msg.height, self.camera_info_msg.width]
        # rospy.loginfo("[Grasp Transform] Camera matrix extraction successful")

        if not self.crop_by_pixel:
            self.crop_size = [int(self.depth_image_size[0]*self.crop_size[0]), int(self.depth_image_size[1]*self.crop_size[1]), 
                              int(self.depth_image_size[0]*self.crop_size[2]), int(self.depth_image_size[1]*self.crop_size[3])]

        # Service that transforms the coordinates
        rospy.Service(self.grasp_in_camera_frame_topic, GraspPrediction, self.transform_coords_cb)

        self.visualisation_topic = rospy.get_param("visualisation")
        self.cropped_rgb_topic = rospy.get_param("cropped_rgb")
        self.cropped_depth_topic = rospy.get_param("cropped_depth")

        # Topic for grasp visualization (Useful for debugging)
        self.img_pub = rospy.Publisher(self.visualisation_topic, Image, queue_size=1)
        self.depth_debug = rospy.Publisher("depth_debug", Image, queue_size=1)

        # Publishes cropped results (Useful for debugging)
        self.rgb_cropped_pub = rospy.Publisher(self.cropped_rgb_topic, Image, queue_size=10)
        self.depth_cropped_pub = rospy.Publisher(self.cropped_depth_topic, Image, queue_size=10) 
        rospy.loginfo("[Image to Camera] Node loaded successfully")

    def _depth_img_callback(self, msg):
        '''
        Subscribes depth image from the corresponding topic 
        '''
        img = self.bridge.imgmsg_to_cv2(msg).copy()

        if not self.waiting:
          return
        
        if self.crop:
            curr_depth_img = img[self.crop_size[0]:self.crop_size[2], self.crop_size[1]:self.crop_size[3]]
            self.depth_cropped_pub.publish(self.bridge.cv2_to_imgmsg(curr_depth_img))
        else:
            curr_depth_img = img 

        if self.remove_noisy_ground_plane:
            max_val = np.mean(curr_depth_img[curr_depth_img > np.percentile(curr_depth_img, 90)]) 
            min_val = np.min(curr_depth_img)

            # Set all depth values in this 1cm range to the same value
            # Assumes the view only has object and ground plane
            curr_depth_img[curr_depth_img > min((min_val + max_val)/2, min_val + self.gripper_height/self.depth_scale)] = max_val
            self.depth_debug.publish(self.bridge.cv2_to_imgmsg(curr_depth_img))

        self.curr_depth_img = curr_depth_img
        self.received = True

    def _rgb_img_callback(self, msg):
        '''
        Subscribes rgb image from the corresponding topic
        '''
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        if not self.waiting:
            return

        if self.crop:
            self.curr_rgb_img = img[self.crop_size[0]:self.crop_size[2], self.crop_size[1]:self.crop_size[3]]
            self.rgb_cropped_pub.publish(self.bridge.cv2_to_imgmsg(self.curr_rgb_img, encoding='rgb8'))
        else:
            self.curr_rgb_img = img

        self.rgb_received = True

    def transform_coords_cb(self, req):
        '''
        Service callback that transforms the coordinates and returns the resulting pose
        '''
        # Waits until a new image is received
        self.waiting = True
      
        # rospy.loginfo("[Grasp Transform] waiting for the next image")
        while not self.received or not self.rgb_received:
            rospy.sleep(0.01)
        # rospy.loginfo("[Grasp Transform] next image received")
    
        # RGB/Depth image is no longer updated
        self.waiting = False
        self.received = False
        self.rgb_received = False

        depth = self.curr_depth_img.copy()
        rgb = self.curr_rgb_img.copy()

        # Perform grasp prediction, Get coords in image frame
        center, width, quality, angle = self.predict_grasp(depth, rgb)

        precrop_center = center.copy() 

        if self.crop:
            # Accounting for crop 
            center[0] = self.crop_size[0] + center[0]
            center[1] = self.crop_size[1] + center[1]
            rospy.loginfo("[Img To Cam] Grasp in Image frame after accounting crop: %s, %s, %s (%s)", center[0], center[1], angle, [self.camera_info_msg.height, self.camera_info_msg.width])
        else:
            rospy.loginfo("[Img To Cam] Grasp in Image frame: %s, %s, %s (%s)", center[0], center[1], angle, [self.camera_info_msg.height, self.camera_info_msg.width])

        # Warping the angle
        angle = (angle + np.pi/2) % np.pi - np.pi/2  # Wrap [-np.pi/2, np.pi/2]

        # check for nearby depths and assign the max of the depths
        # max_z, min_z = self.find_depth_from_rect(depth, int(precrop_center[1]), int(precrop_center[0]), angle)
        # max_z, min_z = max_z*self.depth_scale, min_z*self.depth_scale
        # z = min((min_z + max_z)/2, min_z + 0.03)

        z = self.find_depth_from_gripper_profile(depth, int(precrop_center[1]), int(precrop_center[0]), angle)

        # If you dont want to use the above functionality
        # z = depth[int(precrop_center[0])][int(precrop_center[1])]*self.depth_scale
        
        # TODO u = y, v = x, where x,y are matrix coords and u,v are image coords
        coords_in_cam = np.linalg.inv(self.cam_K)@np.array([[center[1]], [center[0]], [1]])
        coords_in_cam = coords_in_cam*z/coords_in_cam[2][0]

        if not self.angle_2d_in_cam_frame:
            # Transform grasp angle
            angle_vec_in_cam = np.linalg.inv(self.cam_K)@np.array([[np.cos(angle)], [np.sin(angle)], [1]])
            origin_vec_in_cam = np.linalg.inv(self.cam_K)@np.array([[0], [0], [1]])
            
            angle_in_cam = np.arctan2(angle_vec_in_cam[1, 0] - origin_vec_in_cam[1, 0], angle_vec_in_cam[0, 0] - origin_vec_in_cam[0, 0])
        else:
            angle_in_cam = angle

        # Response message
        ret = GraspPredictionResponse()
        ret.success = True
        g = ret.best_grasp
        g.pose.position.x = coords_in_cam[0][0]
        g.pose.position.y = coords_in_cam[1][0]
        g.pose.position.z = coords_in_cam[2][0]
                
        g.pose.orientation = self.list_to_quaternion(tft.quaternion_from_euler(0, 0, angle_in_cam))
        g.width = width
        g.quality = quality

        width = self.get_gripper_width(depth[precrop_center[1], precrop_center[0]]*self.depth_scale)
        if not self.align_depth:
            depth_vis = self.normalize_depth(depth)
            self.draw_angled_rect(depth_vis, precrop_center[1], precrop_center[0], angle, width=width, height=width/2) 
        else:
            self.draw_angled_rect(rgb, precrop_center[1], precrop_center[0], angle, width=width, height=width/2) 

        rospy.loginfo("[Img To Cam] Grasp in camera frame: %s, %s, %s, %s", g.pose.position.x, g.pose.position.y, g.pose.position.z, angle)

        return ret

    def predict_grasp(self, depth, rgb):
        '''
        Calls the service responsible for predicting grasp in the image frame
        '''
        rospy.wait_for_service(self.grasp_in_image_frame_topic, timeout=30)

        try:
            srv_handle = rospy.ServiceProxy(self.grasp_in_image_frame_topic, Grasp2DPrediction)
            
            request_msg = Grasp2DPredictionRequest()
            request_msg.depth_image = self.bridge.cv2_to_imgmsg(depth)
            request_msg.rgb_image = self.bridge.cv2_to_imgmsg(rgb)
            
            response = srv_handle(request_msg)
            center = [response.best_grasp.px, response.best_grasp.py]
            width = response.best_grasp.width
            quality = response.best_grasp.quality
            angle = response.best_grasp.angle

            return center, width, quality, angle

        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)

    def draw_angled_rect(self, image, x, y, angle, width=200, height=100):
        """
        Draws bounding box for visualization
        """
        # Create a rotated rectangle
        angle = (angle - np.pi/2 + np.pi/2) % np.pi - np.pi/2  # Wrap [-np.pi/2, np.pi/2] / Gripper frame is 90 degree off
        angle = angle*180/np.pi
        rect = ((x, y), (width, height), angle)
        color = (255, 0, 0)
        
        # Compute the vertices of the rectangle
        vertices = cv2.boxPoints(rect)
        vertices = np.int0(vertices)
        
        # Draw the rectangle
        image = cv2.drawContours(image, [vertices], 0, color, 2)

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="rgb8"))

    def normalize_depth(self, depth_image):
        normalized_depth_image = ((depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))) * 255
        normalized_depth_image = np.uint8(normalized_depth_image)
        normalized_depth_image = cv2.cvtColor(normalized_depth_image, cv2.COLOR_GRAY2RGB)

        return normalized_depth_image

    def find_depth_from_rect(self, depth_image, x, y, angle, width=180, height=100):
        """
        Finds the top most point inside the bounding box
        x, y are in image pixel coordinates
        """
        # Orientation of the bounding box
        b = np.cos(-angle) * 0.5
        a = np.sin(-angle) * 0.5

        # Corners of the bounding box
        pt0 = (int(x - a * height - b * width), int(y + b * height - a * width))
        pt1 = (int(x + a * height - b * width), int(y - b * height - a * width))
        pt2 = (int(2 * x - pt0[0]), int(2 * y - pt0[1]))
        pt3 = (int(2 * x - pt1[0]), int(2 * y - pt1[1]))

        mask = np.zeros((depth_image.shape), dtype=np.uint8)
        
        pts = np.array( [[[pt0[0], pt0[1]], 
                          [pt1[0], pt1[1]],
                          [pt2[0], pt2[1]],
                          [pt3[0], pt3[1]]]], dtype=np.int32)
        cv2.fillPoly(mask, pts, 1)
        values = depth_image[np.where((mask == 1))]

        return max(values), min(values)
    
    def get_pixels_around_point(self, image_shape, center, radius):
        """
        Get the pixels around a given point based on a pixel radius.
    
        :param image_shape: The shape of the image (height, width).
        :param center: The center point (y, x).
        :param radius: The pixel radius.
        :return: A list of tuples containing the (y, x) coordinates of the pixels within the radius of the center point.
        """
        height, width = image_shape
        x, y = center
        x_coords, y_coords = np.meshgrid(np.arange(width), np.arange(height))
        distances = np.sqrt((x_coords - x) ** 2 + (y_coords - y) ** 2)
        mask = (distances <= radius)
        return np.column_stack((y_coords[mask], x_coords[mask]))
        
    def find_depth_from_gripper_profile(self, depth_image, x, y, angle):
        """
        Finds the z position based on the gripper profile
        TODO: The maximum possible gripper width is considered for the algorithm.

        :param x,y: The center of the gripper in pixel coordinates
        :return z: Optimal z of the detected grasp   
        """
        angle = angle - np.pi/2
        z = depth_image[int(y), int(x)]*self.depth_scale

        width = self.get_gripper_width(z)
        thickness = 1 
        
        point_1_x = x - width/2*np.cos(angle)
        point_1_y = y - width/2*np.sin(angle)
        point_2_x = x + width/2*np.cos(angle)
        point_2_y = y + width/2*np.sin(angle)
                
        point_1 = self.get_pixels_around_point(depth_image.shape, (point_1_x, point_1_y), thickness)
        point_2 = self.get_pixels_around_point(depth_image.shape, (point_2_x, point_2_y), thickness)
        center = self.get_pixels_around_point(depth_image.shape, (x, y), int(width/10))

        try:
            point_1_depth = np.min(depth_image[point_1[:, 0], point_1[:, 1]], axis=0)*self.depth_scale
            point_2_depth = np.min(depth_image[point_2[:, 0], point_2[:, 1]], axis=0)*self.depth_scale
            center_depth = np.min(depth_image[center[:, 0], center[:, 1]], axis=0)*self.depth_scale
        except Exception:
            rospy.logerr("Grasp out of image limits")
            return 500*self.depth_scale 

        return min(center_depth + self.gripper_height - 0.015, point_1_depth - 0.015, point_2_depth - 0.015)
        
    def get_gripper_width(self, z):
        """
        :param z: z of the detected grasp
        Takes the width of the gripper in camera frame and finds the width of the gripper in the image frame 
        """
        width_in_img_frame = self.cam_K@np.array([[self.gripper_width], [0], [z]])
        orig_img_frame = self.cam_K@np.array([[0], [0], [z]])

        width = abs((width_in_img_frame/width_in_img_frame[2, 0] - orig_img_frame/orig_img_frame[2, 0])[0, 0])
        return width

    def list_to_quaternion(self, l):
        q = gmsg.Quaternion()
        q.x = l[0]
        q.y = l[1]
        q.z = l[2]
        q.w = l[3]
        return q
