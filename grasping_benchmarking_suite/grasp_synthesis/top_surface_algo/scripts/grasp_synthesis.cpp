// Including all the header files
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <iostream>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <top_surface_algo/GraspPrediction.h>
#include <top_surface_algo/EFDGrasp.h>
                                  
                                  
// Defining the class for point cloud
class PtCloudClass{  
  public:
    
    PtCloudClass(ros::NodeHandle& nh) : n(nh){
        pub = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 3);
        ros::ServiceServer service = n.advertiseService("/top_surface_grasp_service/predict", &PtCloudClass::getGrasp, this);
        // pt_cloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/roi_points", 5, &PtCloudClass::ptCloudCallback, this);
        pt_cloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/panda_camera/depth/points", 5, &PtCloudClass::ptCloudCallback, this);
        camera_frame = "panda_camera_optical_link";

        cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        ros::spin();
    }
    
    // Callback function to Obtain objectclusters, filtered clouds, Concave hull and grasp points
    void ptCloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
    bool getGrasp(top_surface_algo::GraspPrediction::Request  &req, top_surface_algo::GraspPrediction::Response &res);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_roi(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getObjectClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getPassthroughFilteredClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getConvexHulls(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr calculateHull(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getGrasp(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getEFDGrasp(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds);
    void addCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr);
    void addMarker(int id, float point1_x, float point1_y, float point1_z, float point2_x, float point2_y, float point2_z);
    
  private:
    ros::NodeHandle n;
    ros::Subscriber pt_cloud_sub;
    ros::Publisher pub;
    double centroid_table_z;
    std::string camera_frame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::visualization::PCLVisualizer::Ptr hullViewer;
    pcl::visualization::PCLVisualizer::Ptr graspViewer;
    
};


//Service for requesting object point cloud 
bool PtCloudClass::getGrasp(top_surface_algo::GraspPrediction::Request  &req, top_surface_algo::GraspPrediction::Response &res)
{
    ROS_INFO("Request Recieved");

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obj_clusters = getObjectClusters(cloud);
    if (obj_clusters.size() < 1)
    {
        ROS_INFO("No object clusters found");
        return false;
    }

    pcl::PointCloud<pcl::PointXYZRGB> finalCloud;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> passthroughClusters = getPassthroughFilteredClouds(obj_clusters);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> convexHulls = getConvexHulls(passthroughClusters);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> graspClouds = getEFDGrasp(convexHulls);
    // std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> graspClouds = getGrasp(convexHulls);
    
    if (convexHulls.size() > 0){
        finalCloud = *graspClouds[0];
        for (std::size_t i = 1; i < graspClouds.size(); i++){
            finalCloud += *graspClouds[i];
        }
    }
    
    // Calculating the pose of the Object
    Eigen::Vector3f point1(finalCloud[finalCloud.size()-2].x, finalCloud[finalCloud.size()-2].y, finalCloud[finalCloud.size()-2].z);
    Eigen::Vector3f point2(finalCloud[finalCloud.size()-3].x, finalCloud[finalCloud.size()-3].y, finalCloud[finalCloud.size()-3].z);
    
    // Calculating the Orientation of the Object
    float angle = std::atan2(point1.y() - point2.y(), point1.x() - point2.x());
    float z = 0;
    
    if(centroid_table_z - finalCloud[finalCloud.size()-1].z > 0.04){
        z = 0.03;
    }
    else{
        z = 2*(centroid_table_z - finalCloud[finalCloud.size()-1].z)/3;
    } 
    std::cout << z << " " << centroid_table_z - finalCloud[finalCloud.size()-1].z << std::endl;

    // Grasp pose response 
    res.best_grasp.pose.position.x = finalCloud[finalCloud.size()-1].x;
    res.best_grasp.pose.position.y = finalCloud[finalCloud.size()-1].y;
    res.best_grasp.pose.position.z = finalCloud[finalCloud.size()-1].z + z;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, angle + 1.57);
    res.best_grasp.pose.orientation.w = quat.w();    
    res.best_grasp.pose.orientation.x = quat.x();    
    res.best_grasp.pose.orientation.y = quat.y();    
    res.best_grasp.pose.orientation.z = quat.z();

    // Publish the point cloud for visualization
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(finalCloud, output);
    output.header.frame_id = camera_frame;
    output.header.stamp = ros::Time::now();
    pub.publish(output);
    
    std::cout << "Grasp service success" << std::endl;
    return true;
}


void PtCloudClass::addCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr){
    Eigen::Matrix< float, 4, 1 > centroid;
    pcl::PointXYZ centroidpoint;

    pcl::compute3DCentroid(*CloudPtr, centroid); 
    centroidpoint.x = centroid[0];
    centroidpoint.y = centroid[1];
    centroidpoint.z = centroid[2];

    CloudPtr->push_back(centroidpoint);
}


void PtCloudClass::ptCloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_cloud){
    pcl::fromROSMsg(*in_cloud, *cloud);   
}


// Function to Segment the object cloud from the table plane via RANSAC 
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PtCloudClass::getObjectClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> object_clusters;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    // std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.008f, 0.008f, 0.008f);
    vg.filter (*cloud_filtered);
    // std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);

    int nr_points = (int) cloud_filtered->size ();
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        //Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        if (cloud_f->size() < 1)
        {
            return object_clusters;
        }
        *cloud_filtered = *cloud_f;
    }
    // Computing the centroid of the Table
    Eigen::Vector4f centroid_table;
    pcl::compute3DCentroid(*cloud_plane, centroid_table);
    centroid_table_z = centroid_table[2]; 

    return {cloud_filtered};
}


// Function to filter the Z-axis of the object Cluster
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PtCloudClass::getPassthroughFilteredClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds){
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fl_clouds;
    
    int count = 0;
    for (auto cloud : clouds){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D (*cloud, minPt, maxPt);
        // Specifying the Z-axis threshold limit for filtering(Please do not change) 
        // double z_limit = std::min(0.03, (centroid_table_z + minPt.z)/2 - minPt.z);
        double z_limit = 0.03;

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (minPt.z, minPt.z+z_limit);
        pass.filter (*cloud_filtered);
        
        pcl::getMinMax3D (*cloud_filtered, minPt, maxPt);
        
        Eigen::Vector4f centroid;
        Eigen::Vector4f pcaCentroid;
        Eigen::Matrix3f covariance_matrix;
        pcl::compute3DCentroid(*cloud_filtered, pcaCentroid);

       
        fl_clouds.push_back(cloud_filtered);
        count++;        
    }  
    return fl_clouds;
}


// Function to obtain a Concave Hull 
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PtCloudClass::getConvexHulls(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds){
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fl_clouds;
    
    int count = 0;
    for (auto cloud : clouds){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr cluster_coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr cluster_inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> cluster_seg;
        // Optional
        cluster_seg.setOptimizeCoefficients (true);
        // Mandatory
        cluster_seg.setModelType (pcl::SACMODEL_PLANE);
        cluster_seg.setMethodType (pcl::SAC_RANSAC);
        cluster_seg.setDistanceThreshold (0.01);

        cluster_seg.setInputCloud (cloud);
        cluster_seg.segment (*cluster_inliers, *cluster_coefficients);

        // Project the model inliers
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (cloud);
        proj.setModelCoefficients (cluster_coefficients);
        proj.filter (*cloud_projected);

        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D (*cloud, minPt, maxPt);
   
        for (size_t i = 0; i < cloud_projected->points.size(); ++i)
        {
        cloud_projected->points[i].z = minPt.z;
        }

        /************************* Create a Concave Hull representation of the projected inliers******************/
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        cloud_hull = calculateHull(cloud_projected);
        Eigen::Vector4f centroid;
        Eigen::Vector4f pcaCentroid;
        Eigen::Matrix3f covariance_matrix;
        pcl::compute3DCentroid(*cloud_hull, pcaCentroid);
        
        fl_clouds.push_back(cloud_hull);
        count++;
    }  
    return fl_clouds;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr PtCloudClass::calculateHull(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (CloudPtr);
    chull.setAlpha (0.01);
    chull.reconstruct (*cloud_hull);
    
    return cloud_hull;  
}


std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PtCloudClass::getEFDGrasp(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds){
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fl_clouds;

    int count = 0;
    for (auto CloudPtr : clouds){

        ros::ServiceClient client = n.serviceClient<top_surface_algo::EFDGrasp>("/top_surface_grasp_service/predict_efd_grasp");

        // Create input and output messages
        sensor_msgs::PointCloud2 input_cloud;
        sensor_msgs::PointCloud2 output_cloud;
        
        pcl::toROSMsg(*CloudPtr, input_cloud);
          
        top_surface_algo::EFDGrasp srv;
        srv.request.input_cloud = input_cloud;
        
        if (client.call(srv)){
            // Copy the output message from the service response
            output_cloud = srv.response.output_cloud;
            ROS_INFO("Service call successful");
        }
        else{
            ROS_ERROR("Failed to call service %s", client.getService().c_str());
            return fl_clouds;
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(output_cloud, *pcl_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*pcl_cloud, *cloud_rgb);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vis (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*CloudPtr, *cloud_vis);

        pcl::PointXYZRGB grasppoint;
        grasppoint.x = (pcl_cloud->points[0].x + pcl_cloud->points[1].x)/2; 
        grasppoint.y = (pcl_cloud->points[0].y + pcl_cloud->points[1].y)/2;
        grasppoint.z = (pcl_cloud->points[0].z + pcl_cloud->points[1].z)/2;

        for (size_t i = 0; i < cloud_vis->size(); ++i)
        {
            cloud_vis->points[i].r = 255; 
            cloud_vis->points[i].g = 255;  
            cloud_vis->points[i].b = 255;  
        }
        
        cloud_vis->push_back(cloud_rgb->points[0]);
        cloud_vis->push_back(cloud_rgb->points[1]);
        cloud_vis->push_back(grasppoint);

        cloud_vis->points[cloud_vis->points.size()-1].r = 0;
        cloud_vis->points[cloud_vis->points.size()-1].g = 0;
        cloud_vis->points[cloud_vis->points.size()-1].b = 255;

        cloud_vis->points[cloud_vis->points.size()-2].r = 255;
        cloud_vis->points[cloud_vis->points.size()-2].g = 0;
        cloud_vis->points[cloud_vis->points.size()-2].b = 255;

        cloud_vis->points[cloud_vis->points.size()-3].r = 255;
        cloud_vis->points[cloud_vis->points.size()-3].g = 0;
        cloud_vis->points[cloud_vis->points.size()-3].b = 0;

        fl_clouds.push_back(cloud_vis);
    }
    return fl_clouds;
}


// Obtaining the Grasp Points within the Concave Hull
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PtCloudClass::getGrasp(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds){
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fl_clouds;

    int count = 0;
    for (auto CloudPtr : clouds){
        addCentroid(CloudPtr);
        float min_dis = FLT_MAX;
        int index_closest_point,index_opposite_point;
        // Obtaining Grasp Point 1 (GP1) based on nearest eucledian distance from the Centroid
        for (std::size_t i = 0; i < (CloudPtr->points.size() - 1); ++i)
        {
            float dist_x = CloudPtr->points[(CloudPtr->points.size()-1)].x - CloudPtr->points[i].x;
            float dist_y = CloudPtr->points[(CloudPtr->points.size()-1)].y - CloudPtr->points[i].y;
            float dis = sqrt(dist_x*dist_x + dist_y*dist_y);

            if (dis < min_dis)
            {
                min_dis = dis;
                index_closest_point = i;
            }

        }
        // Calculating Grasp Point 2 (GP2) which is the opposite point of GP1
        pcl::PointXYZ mirrorpoint;
        mirrorpoint.x = (2*CloudPtr->points[(CloudPtr->points.size()-1)].x) - CloudPtr->points[index_closest_point].x;
        mirrorpoint.y = (2*CloudPtr->points[(CloudPtr->points.size()-1)].y) - CloudPtr->points[index_closest_point].y;
        
        min_dis = FLT_MAX;
        for (std::size_t i = 0; i < (CloudPtr->points.size() - 1); ++i)
        {
            float dist_x = mirrorpoint.x - CloudPtr->points[i].x;
            float dist_y = mirrorpoint.y - CloudPtr->points[i].y;
            float dis = sqrt(dist_x*dist_x + dist_y*dist_y);

            if (dis < min_dis)
            {
                min_dis = dis;
                index_opposite_point = i;
            }
        }
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vis (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*CloudPtr, *cloud_vis);

        for (std::size_t i = 0; i < cloud_vis->points.size(); ++i){
            cloud_vis->points[i].r = 255;
            cloud_vis->points[i].b = 255;
            cloud_vis->points[i].g = 255;
        }
        // Choosing Red color to visualize GP1
        cloud_vis->points[index_closest_point].r = 255;
        cloud_vis->points[index_closest_point].b = 0;
        cloud_vis->points[index_closest_point].g = 0;
        // Choosing Blue color to visualize Centroid
        cloud_vis->points[cloud_vis->points.size()-1].r = 0;
        cloud_vis->points[cloud_vis->points.size()-1].g = 0;
        cloud_vis->points[cloud_vis->points.size()-1].b = 255;
        // Choosing Red color to visualize GP2
        cloud_vis->points[index_opposite_point].r = 255;
        cloud_vis->points[index_opposite_point].b = 0;
        cloud_vis->points[index_opposite_point].g = 0;

        pcl::PointXYZRGB grasppoint;
        grasppoint.x = (cloud_vis->points[index_closest_point].x + cloud_vis->points[index_opposite_point].x)/2;
        grasppoint.y = (cloud_vis->points[index_closest_point].y + cloud_vis->points[index_opposite_point].y)/2;
        grasppoint.z = (cloud_vis->points[index_closest_point].z + cloud_vis->points[index_opposite_point].z)/2;

        cloud_vis->push_back(cloud_vis->points[index_closest_point]);
        cloud_vis->push_back(cloud_vis->points[index_opposite_point]);
        cloud_vis->push_back(grasppoint);
        
        fl_clouds.push_back(cloud_vis);
        count++;
    }    
    return fl_clouds;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_synthesis_node");
  ros::NodeHandle n;
  PtCloudClass PtCloudObj(n);
  return 0;
}