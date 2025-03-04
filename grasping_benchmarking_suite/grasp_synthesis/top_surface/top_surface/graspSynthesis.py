#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from grasp_interfaces.srv import GraspPrediction, EFDGrasp
import tf_transformations
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import PointCloud2, PointField
from grasp_interfaces.srv import EFDGrasp
from sensor_msgs.msg import PointField, Image
from sensor_msgs_py.point_cloud2 import read_points, create_cloud
from utils import get_grasp
from rclpy.executors import MultiThreadedExecutor
from threading import Lock
import threading

# def pointcloud2_to_open3d(cloud_msg: PointCloud2) -> o3d.geometry.PointCloud:
#     """
#     Convert a ROS2 PointCloud2 message to an Open3D point cloud.
#     Assumes the cloud contains only XYZ float32 data.
#     """
#     points = []
#     # Read x, y, z fields; skip NaNs.
#     for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
#         points.append([p[0], p[1], p[2]])
#     pcd = o3d.geometry.PointCloud()
#     if points:
#         pcd.points = o3d.utility.Vector3dVector(np.array(points))
#     return pcd

# def open3d_to_pointcloud2(pcd: o3d.geometry.PointCloud, frame_id="panda_camera_optical_link") -> PointCloud2:
#     """
#     Convert an Open3D point cloud to a ROS2 PointCloud2 message.
#     Only writes the x, y, z fields.
#     """
#     points = np.asarray(pcd.points)
#     # Create header with current time.
#     current_time = rclpy.clock.Clock().now().to_msg()
#     header = Header()
#     header.stamp = current_time
#     header.frame_id = frame_id
#     fields = [
#         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
#     ]
#     cloud_msg = pc2.create_cloud(header, fields, points)
#     return cloud_msg

# def combine_point_clouds(cloud_list):
#     """Combine a list of Open3D point clouds into one."""
#     combined = o3d.geometry.PointCloud()
#     all_points = []
#     all_colors = []
#     for cloud in cloud_list:
#         pts = np.asarray(cloud.points)
#         all_points.append(pts)
#         if cloud.has_colors():
#             cols = np.asarray(cloud.colors)
#         else:
#             cols = np.ones((pts.shape[0], 3))  # default to white
#         all_colors.append(cols)
#     if all_points:
#         combined.points = o3d.utility.Vector3dVector(np.vstack(all_points))
#         combined.colors = o3d.utility.Vector3dVector(np.vstack(all_colors))
#     return combined

class PtCloudNode(Node):
    def __init__(self):
        super().__init__('grasp_synthesis_node')
        self.cb_grp_rentrant = ReentrantCallbackGroup()
        self.cb_grp_mut = MutuallyExclusiveCallbackGroup()
        # Subscriber to point cloud topic.
        self.subscription = self.create_subscription(
            PointCloud2,
            "/panda_camera/depth/points",
            self.pt_cloud_callback,
            5,
            callback_group=self.cb_grp_mut)
        # Publisher for the filtered/grasp cloud.
        self.publisher_ = self.create_publisher(PointCloud2, "filtered_cloud", 3, callback_group=self.cb_grp_mut)
        # Service for grasp prediction.
        # self.srv = self.create_service(GraspPrediction, "/top_surface_grasp_service/predict", self.get_grasp_callback, callback_group=self.cb_grp_rentrant)
        self.cloud = None  # Will hold the latest Open3D point cloud.
        self.centroid_table_z = 0.0
        self.camera_frame = "panda_camera_optical_link"
        self.get_logger().info("Grasp synthesis node initialized.")
        self.lock = Lock()

    def init_service(self):
        self.srv = self.create_service(GraspPrediction, "/top_surface_grasp_service/predict", self.get_grasp_callback, callback_group=self.cb_grp_rentrant)

    def pt_cloud_callback(self, msg: PointCloud2):
        self.get_logger().info("Received point cloud message.")
        with self.lock:
            self.cloud = self.pointcloud2_to_open3d(msg)

    def handle_get_grasp(self, request, response):
        self.get_logger().error("SERVICE REQUEST RECEIVED")
        '''
        Service callback for grasp point calculation
        '''
        self.get_logger().info("Received grasp request")
        
        # Read point cloud data properly using field names
        points = list(read_points(request.input_cloud, field_names=("x", "y", "z"), skip_nans=True))
        
        if not points:
            self.get_logger().error("Empty point cloud received")
            return response

        # Convert to numpy array with proper shape (N, 3)
        point_cloud = np.array([(p[0], p[1], p[2]) for p in points])
        
        self.get_logger().info(f"Point Cloud shape: {point_cloud.shape}")
        
        try:
            z_mean = np.mean(point_cloud[:, 2])
            self.get_logger().info("Mean found")
            
            # Use only x,y coordinates for grasp calculation
            grasp = get_grasp(point_cloud[:, :2], visualize=True, split=True)
            self.get_logger().info("grasp found")
            
            # Add z-coordinate back to grasp points
            grasp = np.hstack((grasp, np.ones((grasp.shape[0], 1)) * z_mean))
            
            header = request.input_cloud.header
            header.stamp = self.get_clock().now().to_msg()

            # Create fields for the PointCloud2 message
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]

            # Create the PointCloud2 message
            response.output_cloud = create_cloud(header, fields, grasp)
            self.get_logger().info("point_cloud created")
            
        except Exception as e:
            self.get_logger().error(f"Error processing grasp: {str(e)}")
            # return response

        # return response

    # def get_grasp_callback(self, request, response):
    #     self.get_logger().info("Grasp service request received.")
    #     with self.lock:
    #         if self.cloud is None or len(self.cloud.points) == 0:
    #             self.get_logger().error("No point cloud data available.")
    #             return response

    #     obj_clusters = None
    #     # Segment the objects from the table.
    #     with self.lock:
    #         obj_clusters = self.get_object_clusters(self.cloud)
    #     if len(obj_clusters) < 1:
    #         self.get_logger().info("No object clusters found")
    #         return response
        
    def get_grasp_callback(self, request, response):
        self.get_logger().info("Grasp service request received.")
        
        # First check if we have point cloud data
        with self.lock:
            if self.cloud is None or len(self.cloud.points) == 0:
                self.get_logger().error("No point cloud data available.")
                response.success = False
                return response
            
            # Make a local copy to work with outside the lock
            local_cloud = o3d.geometry.PointCloud()
            local_cloud.points = o3d.utility.Vector3dVector(np.asarray(self.cloud.points))
        
        # Process the point cloud data outside the lock
        try:
            # Process using local_cloud instead of self.cloud
            obj_clusters = self.get_object_clusters(local_cloud)
            if len(obj_clusters) < 1:
                self.get_logger().info("No object clusters found")
                response.success = False
                return response

            # Filter the object clusters.
            passthrough_clusters = self.get_passthrough_filtered_clouds(obj_clusters)
            # Obtain the projected hulls.
            convex_hulls = self.get_convex_hulls(passthrough_clusters)
            # Use the EFD grasp service to generate grasp clouds.
            grasp_clouds = self.get_efd_grasp(convex_hulls)
            final_cloud = o3d.geometry.PointCloud()
            if len(grasp_clouds) > 0:
                # Combine all grasp clouds.
                final_cloud = self.combine_point_clouds(grasp_clouds)
            else:
                self.get_logger().error("No grasp clouds generated.")
                return response

            final_points = np.asarray(final_cloud.points)
            if final_points.shape[0] < 3:
                self.get_logger().error("Not enough points in final cloud for grasp calculation.")
                return response

            # Use the second-last and third-last points to compute the grasp orientation.
            point1 = final_points[-2]
            point2 = final_points[-3]
            last_point = final_points[-1]
            angle = math.atan2(point1[1] - point2[1], point1[0] - point2[0])
            # Calculate a z-offset based on the table centroid.
            if (self.centroid_table_z - last_point[2]) > 0.04:
                z_offset = 0.03
            else:
                z_offset = 2 * (self.centroid_table_z - last_point[2]) / 3
            self.get_logger().info("Z offset: {}  Diff: {}".format(z_offset, self.centroid_table_z - last_point[2]))

            # Fill in the grasp pose in the service response.
            response.best_grasp.pose.position.x = float(last_point[0])
            response.best_grasp.pose.position.y = float(last_point[1])
            response.best_grasp.pose.position.z = float(last_point[2] + z_offset)

            # Create a quaternion from the computed yaw (adding 1.57 rad).
            q = tf_transformations.quaternion_from_euler(0, 0, angle + 1.57)
            response.best_grasp.pose.orientation.x = q[0]
            response.best_grasp.pose.orientation.y = q[1]
            response.best_grasp.pose.orientation.z = q[2]
            response.best_grasp.pose.orientation.w = q[3]

            # Publish the final (grasp) cloud for visualization.
            ros_cloud = self.open3d_to_pointcloud2(final_cloud, frame_id=self.camera_frame)
            self.publisher_.publish(ros_cloud)
            self.get_logger().info("Grasp service succeeded.")
            response.success = True
        
        except Exception as e:
            self.get_logger().error(f"Error in grasp processing: {str(e)}")
            response.success = False
            
        return response

    def get_object_clusters(self, cloud: o3d.geometry.PointCloud):
        """
        Downsamples the cloud and removes the table plane using RANSAC.
        Returns a list containing the remaining object cluster.
        """
        voxel_size = 0.008
        cloud_down = cloud.voxel_down_sample(voxel_size)
        original_count = len(cloud_down.points)
        cloud_filtered = cloud_down
        plane_cloud = None

        # Remove large planar components (assumed to be the table).
        while len(cloud_filtered.points) > 0.3 * original_count:
            plane_model, inliers = cloud_filtered.segment_plane(distance_threshold=0.01,
                                                                  ransac_n=3,
                                                                  num_iterations=100)
            if len(inliers) == 0:
                break
            plane_cloud = cloud_filtered.select_by_index(inliers)
            cloud_filtered = cloud_filtered.select_by_index(inliers, invert=True)
            if len(cloud_filtered.points) < 1:
                break

        if plane_cloud is not None and len(plane_cloud.points) > 0:
            pts = np.asarray(plane_cloud.points)
            centroid = np.mean(pts, axis=0)
            self.centroid_table_z = centroid[2]
        return [cloud_filtered]

    def get_passthrough_filtered_clouds(self, clouds):
        """
        Filters each cloud along the z-axis to keep points within a threshold (0.03 m).
        """
        filtered_clouds = []
        for cloud in clouds:
            pts = np.asarray(cloud.points)
            if pts.size == 0:
                continue
            min_z = np.min(pts[:, 2])
            z_limit = 0.03
            mask = (pts[:, 2] >= min_z) & (pts[:, 2] <= min_z + z_limit)
            filtered_pts = pts[mask]
            if filtered_pts.shape[0] == 0:
                continue
            cloud_filtered = o3d.geometry.PointCloud()
            cloud_filtered.points = o3d.utility.Vector3dVector(filtered_pts)
            filtered_clouds.append(cloud_filtered)
        return filtered_clouds

    def get_convex_hulls(self, clouds):
        """
        Projects the cloud onto a plane and computes a concave (alpha shape) hull.
        """
        hulls = []
        for cloud in clouds:
            pts = np.asarray(cloud.points)
            if pts.size == 0:
                continue
            pts_proj = pts.copy()
            min_z = np.min(pts_proj[:, 2])
            pts_proj[:, 2] = min_z  # project onto the plane at min z
            projected_cloud = o3d.geometry.PointCloud()
            projected_cloud.points = o3d.utility.Vector3dVector(pts_proj)
            hull = self.calculate_hull(projected_cloud)
            hulls.append(hull)
        return hulls

    # def calculate_hull(self, cloud: o3d.geometry.PointCloud):
    #     """
    #     Calculates a concave hull using an alpha shape and returns its vertices as a point cloud.
    #     """
    #     try:
    #         mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(cloud, alpha=0.01)
    #         mesh.compute_vertex_normals()
    #         vertices = np.asarray(mesh.vertices)
    #         hull_cloud = o3d.geometry.PointCloud()
    #         hull_cloud.points = o3d.utility.Vector3dVector(vertices)
    #         return hull_cloud
    #     except Exception as e:
    #         self.get_logger().error("Error in calculating hull: {}".format(e))
    #         return cloud

    def calculate_hull(self, cloud: o3d.geometry.PointCloud):
        """
        Calculates a concave hull using an alpha shape.
        A small jitter is added to the points to overcome Qhull precision issues.
        """
        pts = np.asarray(cloud.points)
        if pts.size == 0:
            return cloud
        # Add a small random noise to avoid cocircular/cospherical configurations.
        jitter = np.random.normal(0, 1e-4, pts.shape)
        pts_jittered = pts + jitter
        cloud_jittered = o3d.geometry.PointCloud()
        cloud_jittered.points = o3d.utility.Vector3dVector(pts_jittered)
        
        try:
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(cloud_jittered, alpha=0.01)
            mesh.compute_vertex_normals()
            vertices = np.asarray(mesh.vertices)
            hull_cloud = o3d.geometry.PointCloud()
            hull_cloud.points = o3d.utility.Vector3dVector(vertices)
            return hull_cloud
        except Exception as e:
            self.get_logger().error("Error in calculating hull: {}".format(e))
            return cloud

    def get_efd_grasp(self, clouds):
        """
        For each cloud, calls the EFDGrasp service to obtain a grasp prediction.
        Then, it creates a visualization point cloud that marks the grasp points with colors.
        """
        grasp_clouds = []
        for cloud in clouds:
            # client = self.create_client(EFDGrasp, "/top_surface_grasp_service/predict")
            # if not client.wait_for_service(timeout_sec=1.0):
            #     self.get_logger().error("EFDGrasp service not available.")
            #     continue
            req = EFDGrasp.Request()
            req.input_cloud = self.open3d_to_pointcloud2(cloud, frame_id=self.camera_frame)
            # future = client.call_async(req)
            # rclpy.spin_until_future_complete(self, future)
            # if future.result() is None:
            #     self.get_logger().error("Failed to call EFDGrasp service.")
            #     continue
            # output_cloud_msg = future.result().output_cloud
            response = EFDGrasp.Response()
            self.handle_get_grasp(req, response)
            output_cloud_msg = response.output_cloud
            # Convert the service output to an Open3D point cloud.
            pcl_cloud = self.pointcloud2_to_open3d(output_cloud_msg)
            # Create a colored version (RGB) of the output.
            cloud_rgb = pcl_cloud
            # Create a copy of the original cloud for visualization.
            cloud_vis = o3d.geometry.PointCloud()
            pts = np.asarray(cloud.points)
            cloud_vis.points = o3d.utility.Vector3dVector(pts)
            # Set all points to white.
            cloud_vis.colors = o3d.utility.Vector3dVector(np.ones((pts.shape[0], 3)))
            # Compute the grasp point as the average of the first two points from the service output.
            pcl_pts = np.asarray(pcl_cloud.points)
            if pcl_pts.shape[0] < 2:
                self.get_logger().error("Not enough points in EFDGrasp output.")
                continue
            grasp_point = (pcl_pts[0] + pcl_pts[1]) / 2.0

            # Prepare appended points and assign colors:
            # - First appended point: red (grasp point 1)
            # - Second appended point: magenta (grasp point 2)
            # - Third appended point: blue (average grasp point)
            appended_pts = np.array([pcl_pts[0], pcl_pts[1], grasp_point])
            appended_colors = np.array([[1, 0, 0], [1, 0, 1], [0, 0, 1]])
            vis_pts = np.vstack((np.asarray(cloud_vis.points), appended_pts))
            vis_colors = np.vstack((np.asarray(cloud_vis.colors), appended_colors))
            cloud_vis.points = o3d.utility.Vector3dVector(vis_pts)
            cloud_vis.colors = o3d.utility.Vector3dVector(vis_colors)

            grasp_clouds.append(cloud_vis)
        return grasp_clouds

    def get_grasp_from_clouds(self, clouds):
        """
        (Alternative non-EFD grasp calculation.)
        For each cloud, computes the centroid, finds the nearest point (GP1), and then a mirror (GP2),
        colors them for visualization, and appends the points.
        """
        grasp_clouds = []
        for cloud in clouds:
            self.add_centroid(cloud)
            pts = np.asarray(cloud.points)
            if pts.shape[0] < 2:
                continue
            centroid = pts[-1]
            # Find the point closest to the centroid (ignoring the appended centroid).
            distances = np.linalg.norm(pts[:-1] - centroid, axis=1)
            index_closest = np.argmin(distances)
            gp1 = pts[index_closest]
            # Compute the mirror point.
            mirror_point = 2 * centroid - gp1
            distances_mirror = np.linalg.norm(pts[:-1] - mirror_point, axis=1)
            index_opposite = np.argmin(distances_mirror)

            # Create a visualization copy.
            cloud_vis = o3d.geometry.PointCloud()
            cloud_vis.points = o3d.utility.Vector3dVector(pts.copy())
            cloud_vis.colors = o3d.utility.Vector3dVector(np.ones((pts.shape[0], 3)))
            colors = np.asarray(cloud_vis.colors)
            colors[index_closest] = [1, 0, 0]   # GP1 in red.
            colors[-1] = [0, 0, 1]              # Centroid in blue.
            colors[index_opposite] = [1, 0, 0]    # GP2 in red.
            cloud_vis.colors = o3d.utility.Vector3dVector(colors)
            grasp_point = (np.asarray(cloud_vis.points)[index_closest] +
                           np.asarray(cloud_vis.points)[index_opposite]) / 2.0
            appended_pts = np.array([np.asarray(cloud_vis.points)[index_closest],
                                     np.asarray(cloud_vis.points)[index_opposite],
                                     grasp_point])
            appended_colors = np.array([[1, 0, 0], [1, 0, 0], [0, 0, 1]])
            vis_pts = np.vstack((np.asarray(cloud_vis.points), appended_pts))
            vis_colors = np.vstack((np.asarray(cloud_vis.colors), appended_colors))
            cloud_vis.points = o3d.utility.Vector3dVector(vis_pts)
            cloud_vis.colors = o3d.utility.Vector3dVector(vis_colors)
            grasp_clouds.append(cloud_vis)
        return grasp_clouds

    def add_centroid(self, cloud: o3d.geometry.PointCloud):
        """
        Computes the centroid of the cloud and appends it as an extra point.
        """
        pts = np.asarray(cloud.points)
        if pts.size == 0:
            return
        centroid = np.mean(pts, axis=0)
        pts = np.vstack((pts, centroid))
        cloud.points = o3d.utility.Vector3dVector(pts)

    def pointcloud2_to_open3d(self, cloud_msg: PointCloud2) -> o3d.geometry.PointCloud:
        """
        Convert a ROS2 PointCloud2 message to an Open3D point cloud.
        Assumes the cloud contains only XYZ float32 data.
        """
        points = []
        # Read x, y, z fields; skip NaNs.
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        pcd = o3d.geometry.PointCloud()
        if points:
            pcd.points = o3d.utility.Vector3dVector(np.array(points))
        return pcd

    def open3d_to_pointcloud2(self, pcd: o3d.geometry.PointCloud, frame_id="panda_camera_optical_link") -> PointCloud2:
        """
        Convert an Open3D point cloud to a ROS2 PointCloud2 message.
        Only writes the x, y, z fields.
        """
        points = np.asarray(pcd.points)
        # Create header with current time.
        current_time = rclpy.clock.Clock().now().to_msg()
        header = Header()
        header.stamp = current_time
        header.frame_id = frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        cloud_msg = pc2.create_cloud(header, fields, points)
        return cloud_msg

    def combine_point_clouds(self, cloud_list):
        """Combine a list of Open3D point clouds into one."""
        combined = o3d.geometry.PointCloud()
        all_points = []
        all_colors = []
        for cloud in cloud_list:
            pts = np.asarray(cloud.points)
            all_points.append(pts)
            if cloud.has_colors():
                cols = np.asarray(cloud.colors)
            else:
                cols = np.ones((pts.shape[0], 3))  # default to white
            all_colors.append(cols)
        if all_points:
            combined.points = o3d.utility.Vector3dVector(np.vstack(all_points))
            combined.colors = o3d.utility.Vector3dVector(np.vstack(all_colors))
        return combined

def init_srv(node:PtCloudNode):
    node.init_service()

def main(args=None):
    rclpy.init(args=args)
    node = PtCloudNode()
    try:
        init_srv_thread = threading.Thread(target=init_srv, args=(node,))
        init_srv_thread.start()
        rclpy.spin(node, executor=MultiThreadedExecutor())
        init_srv.join()
        # rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down grasp synthesis node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
