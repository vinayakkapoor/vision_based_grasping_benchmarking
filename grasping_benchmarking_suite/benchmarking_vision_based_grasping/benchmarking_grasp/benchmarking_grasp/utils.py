import cv2
import numpy as np
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
import yaml
import os

def generate_experiments(parsed_experiments):
    """
    Generates experiment configurations with object names and calculated poses.
    """
    experiments = []

    for experiment_idx, experiment_data in enumerate(parsed_experiments):
        # print(f"Processing experiment {experiment_idx}")

        # This will now be the list of object names directly
        object_names = experiment_data[0]
        center = experiment_data[1]
        r = experiment_data[2]
        alpha = experiment_data[3]
        n = experiment_data[4]
        height = experiment_data[5]

        center_coord = np.array([center, 0, height, 0, 0, 0])

        # Generate object poses for spawning (relative to center)
        poses = [            
            center_coord,
            center_coord + np.array([r, 0, 0, 0, 0, 0]),
            center_coord + np.array([0, r, 0, 0, 0, 0]),
            center_coord + np.array([0, -r, 0, 0, 0, 0]),  
            center_coord + np.array([0, r, 0, 0, 0, alpha]),             
            center_coord + np.array([0, -r, 0, 0, 0, -alpha]),         
        ]

        # Append the list of object names, the calculated poses, and n
        experiments.append([object_names, poses, n])

    return experiments

def parse_experiments_config(experiments_config_path): 
    """
    Parse information from the yaml file, keeping only object names.
    """
    print("Parsing")
    model_file = open(experiments_config_path, 'r')
    config = yaml.load(model_file, Loader=yaml.FullLoader)
    model_file.close() 

    if config.get('only_first', False): 
        experiments_n = 1
    else:
        # Calculate number of experiments based on keys like 'experiment_X'
        experiment_keys = [k for k in config.keys() if k.startswith('experiment_')]
        experiments_n = len(experiment_keys)

    experiments = []
    for experiment_idx in range(1, experiments_n + 1):
        experiment_key = f"experiment_{experiment_idx}"
        if experiment_key not in config:
            print(f"Warning: Configuration for {experiment_key} not found. Skipping.")
            continue

        object_names = config[experiment_key]["objects"]

        # Get config parameters
        exp_config = config[experiment_key].get("config", {}) # Use .get for safer access
        center = exp_config.get("center", 0.0) # Provide defaults if needed
        r = exp_config.get("r", 0.0)
        alpha = exp_config.get("alpha", 0.0)
        n = exp_config.get("n", 1)
        height = exp_config.get("height", 0.0)

        # Append the list of object names and config parameters
        experiments.append([object_names, center, r, alpha, n, height])

    return experiments

def init_params(node:Node):
    node.declare_parameters(
        namespace='',
        parameters=[
            # General params
            ('sim_mode', True),
            ('use_cartesian', True),
            ('over_head', False),
            ('enable_crop', True),
            ('crop_by_pixel', True),
            ('remove_noisy_ground_plane', False),
            ('angle_2d_in_cam_frame', False),
            ('enable_benchmark_test', True),
            ('point_cloud_input', False),

            # Gripper/robot params
            ('gripper_offset', 0.105),
            ('intermediate_z_stop', 0.5),
            ('scan_pose', [0.45, 0.0, 0.8, 3.14, 0.0, 0.0]),
            ('bad_grasp_z', 0.1),
            ('grasp_angle_offset', 1.57),
            ('x_offset_world', 0.0),
            ('y_offset_world', 0.0),
            ('gripper_width', 0.09),
            ('gripper_height', 0.04),

            # Camera params
            ('depth_crop_size', [17, 17, 1003, 1013]),
            ('pc_roi', [-1.0, -0.24, -0.3, 1.0, 0.28, 0.36]),

            # Robot control params
            ('robot_default_velocity', 0.1),
            ('robot_approach_velocity', 0.5),
            ('robot_benchmarking_velocity', 0.5),

            # Miscellaneous params
            ('record_video', True),
            ('camera_name', "usb-046d_HD_Pro_Webcam_C920_0F85DE4F-video-index0"),
            ('urdf_package_name', 'benchmarking_grasp'),
            ('experiments_package_name', 'benchmarking_grasp'),
            ('experiments_config_relative_path', 'config/benchmarking_experiments.yaml'),

            # Coordinate frames
            ('base_frame', 'panda_link0'),
            ('camera_frame_sim', 'panda_camera_optical_link'),
            ('camera_frame', 'camera_depth_optical_frame'),

            # ROS topics
            ('grasp_in_image_frame', "/ggcnn_grasp_service/predict"),
            ('grasp_in_camera_frame', "coords_in_cam"),
            ('grasp_in_world_frame', "predict"),
            ('cam_info_sim', '/panda_camera/rgb/camera_info'),
            ('depth_image_sim', '/panda_camera/depth/image_raw'),
            ('rgb_image_sim', '/panda_camera/rgb/image_raw'),
            ('point_cloud_sim', "/panda_camera/depth/points"),

            ('cam_info_depth_align', '/camera/aligned_depth_to_color/camera_info'),
            ('cam_info_depth', '/camera/depth/camera_info'),
            ('point_cloud', "/camera/depth/color/points"),
            ('depth_wo_align_image', '/camera/depth/image_rect_raw'),
            ('depth_image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb_image', '/camera/color/image_raw'),

            ('roi_point_cloud', "/camera/depth/color/roi_points"),
            ('depth_complete_image', '/camera/aligned_depth_to_color/depth_completed'),
            ('depth_complete_image_norm', '/camera/aligned_depth_to_color/depth_completed_norm'),

            ('visualisation', "visualisation"),
            ('visualisation_type', 'rgb'),
            ('cropped_depth', "cropped_depth")
        ]
    )

def calculate_camera_frame_transform(center, precrop_center, angle, depth_image, cam_K, 
                                    depth_scale, angle_2d_flag, gripper_width, gripper_height, node):
    """
    Calculates 3D grasp pose in camera frame from 2D image coordinates
    
    Args:
        center: Grasp center in full image coordinates [x,y]
        precrop_center: Grasp center in cropped image coordinates [x,y]
        angle: Grasp angle in image frame (radians)
        depth_image: Depth image array
        cam_K: Camera matrix (3x3)
        depth_scale: Depth scaling factor to meters
        angle_2d_flag: Whether to keep angle in image frame
        gripper_width: Physical gripper width (m)
        gripper_height: Physical gripper height (m)
        
    Returns:
        tuple: (position_xyz, angle_in_cam, gripper_width_px)
    """
    # Calculate depth
    z = find_depth_from_gripper_profile(
        depth_image, 
        int(precrop_center[1]), 
        int(precrop_center[0]), 
        angle,
        depth_scale,
        cam_K,
        gripper_width,
        gripper_height
    )

    # Transform image coordinates to 3D camera frame
    coords_in_cam = np.linalg.inv(cam_K) @ np.array([[center[1]], [center[0]], [1]])
    coords_in_cam = coords_in_cam * z / coords_in_cam[2][0]

    # Calculate angle in camera frame
    if not angle_2d_flag:
        angle_vec = np.linalg.inv(cam_K) @ np.array([[np.cos(angle)], [np.sin(angle)], [1]])
        origin_vec = np.linalg.inv(cam_K) @ np.array([[0], [0], [1]])
        angle_in_cam = np.arctan2(
            angle_vec[1,0] - origin_vec[1,0], 
            angle_vec[0,0] - origin_vec[0,0]
        )
    else:
        angle_in_cam = angle
    
    node.get_logger().info(str([precrop_center[1], precrop_center[0]]))
    # Calculate gripper width in pixels
    width_px = get_gripper_width_cam2img(
        cam_K,z,depth_image[precrop_center[0], precrop_center[1]] * depth_scale, node=node
    )

    return (
        [coords_in_cam[0][0], coords_in_cam[1][0], coords_in_cam[2][0]],
        angle_in_cam,
        width_px
    )

def draw_angled_rect(image, x, y, angle, width=200, height=100):
    """
    Draws bounding box for visualization
    """
    angle = (angle - np.pi/2 + np.pi/2) % np.pi - np.pi/2
    angle = angle*180/np.pi
    rect = ((x, y), (width, height), angle)
    color = (255, 0, 0)
    
    vertices = cv2.boxPoints(rect)
    vertices = np.int0(vertices)
    
    image = cv2.drawContours(image, [vertices], 0, color, 2)
    return image
    # msg = node.bridge.cv2_to_imgmsg(image, encoding="rgb8")
    # msg.header.stamp = node.get_clock().now().to_msg()  # Set timestamp
    # msg.header.frame_id = "map"  # Change this to the correct frame ID
    # node.img_pub.publish(msg)

def normalize_depth(depth_image):
    normalized_depth_image = ((depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))) * 255
    normalized_depth_image = np.uint8(normalized_depth_image)
    normalized_depth_image = cv2.cvtColor(normalized_depth_image, cv2.COLOR_GRAY2RGB)

    return normalized_depth_image

def find_depth_from_rect(depth_image, x, y, angle, width=180, height=100):
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

def get_pixels_around_point(image_shape, center, radius):
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

def get_gripper_width_cam2img(K, z, gripper_width_cam, node=None):
    """
    :param gripper_width: Real Gripper_width in cam frame
    :param K: Camera matrix
    :param z: z of the detected grasp
    Takes the width of the gripper in camera frame and finds the width of the gripper in the image frame 
    """
    
    width_in_img_frame = K @ np.array([[gripper_width_cam], [0], [z]])
    orig_img_frame = K @np.array([[0], [0], [z]])

    width = abs((width_in_img_frame/width_in_img_frame[2, 0] - orig_img_frame/orig_img_frame[2, 0])[0, 0])

    return width

def find_depth_from_gripper_profile(depth_image, x, y, angle, depth_scale, K, gripper_width_cam, gripper_height):
    """
    Finds the z position based on the gripper profile
    TODO: The maximum possible gripper width is considered for the algorithm.

    :param x,y: The center of the gripper in pixel coordinates
    :return z: Optimal z of the detected grasp   
    """
    angle = angle - np.pi/2
    z = depth_image[int(y), int(x)]*depth_scale

    width = get_gripper_width_cam2img(K, z, gripper_width_cam)
    thickness = 1 
    
    point_1_x = x - width/2*np.cos(angle)
    point_1_y = y - width/2*np.sin(angle)
    point_2_x = x + width/2*np.cos(angle)
    point_2_y = y + width/2*np.sin(angle)
            
    point_1 = get_pixels_around_point(depth_image.shape, (point_1_x, point_1_y), thickness)
    point_2 = get_pixels_around_point(depth_image.shape, (point_2_x, point_2_y), thickness)
    center = get_pixels_around_point(depth_image.shape, (x, y), int(width/10))

    try:
        point_1_depth = np.min(depth_image[point_1[:, 0], point_1[:, 1]], axis=0)*depth_scale
        point_2_depth = np.min(depth_image[point_2[:, 0], point_2[:, 1]], axis=0)*depth_scale
        center_depth = np.min(depth_image[center[:, 0], center[:, 1]], axis=0)*depth_scale
    except Exception:
        print("!!!!Grasp out of image limits")
        return 500*depth_scale 

    return min(center_depth + gripper_height - 0.015, point_1_depth - 0.015, point_2_depth - 0.015)

def get_gripper_width(K, z, gripper_width_cam):
    """
    :param gripper_width_cam: Real gripper width in the camera frame
    :param K: camera Matrix
    :param z: z of the detected grasp
    Takes the width of the gripper in camera frame and finds the width of the gripper in the image frame 
    """
    width_in_img_frame = K @ np.array([[gripper_width_cam], [0], [z]])
    orig_img_frame = K @ np.array([[0], [0], [z]])

    width = abs((width_in_img_frame/width_in_img_frame[2, 0] - orig_img_frame/orig_img_frame[2, 0])[0, 0])
    return width

def list_to_quaternion(quat_list):
    """
    Converts a list [x, y, z, w] into a geometry_msgs/Quaternion message.
    """
    q = Quaternion()
    q.x = quat_list[0]
    q.y = quat_list[1]
    q.z = quat_list[2]
    q.w = quat_list[3]
    return q