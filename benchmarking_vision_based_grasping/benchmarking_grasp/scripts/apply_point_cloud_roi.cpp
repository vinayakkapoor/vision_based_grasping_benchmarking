#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class GetPointCloudROI{
    public:
        GetPointCloudROI(ros::NodeHandle& nh);
        void ptCloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyPassthroughFilter(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);

    private:
        ros::NodeHandle n;
        ros::Subscriber pt_cloud_sub;
        ros::Publisher pub;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        float x_min, x_max, y_min, y_max, z_min, z_max;
        std::string camera_frame;
};

GetPointCloudROI::GetPointCloudROI(ros::NodeHandle& nh): n(nh){
    std::string pc_topic, filtered_pc_topic;
    bool sim_mode;
    std::vector<double> roi;
    
    n.getParam("pc_roi", roi);
    n.getParam("sim_mode", sim_mode);
    n.getParam("roi_point_cloud", filtered_pc_topic);        
    if(sim_mode){
        n.getParam("point_cloud_sim", pc_topic);
        n.getParam("camera_frame_sim", camera_frame);
    }
    else{
        n.getParam("point_cloud", pc_topic);
        n.getParam("camera_frame", camera_frame);
    }
        
    z_min = roi[0];
    y_min = roi[1];
    x_min = roi[2];
    z_max = roi[3];
    y_max = roi[4];
    x_max = roi[5];

    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pt_cloud_sub = n.subscribe<sensor_msgs::PointCloud2>(pc_topic, 1, &GetPointCloudROI::ptCloudCallback, this);
    pub = n.advertise<sensor_msgs::PointCloud2>(filtered_pc_topic, 1);
    
    ros::spin();
}

void GetPointCloudROI::ptCloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_cloud){
    pcl::fromROSMsg(*in_cloud, *cloud);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthroughCloud = applyPassthroughFilter(cloud, x_min, x_max, y_min, y_max, z_min, z_max);
    
    sensor_msgs::PointCloud2 out_cloud;
    pcl::toROSMsg(*passthroughCloud, out_cloud);
    out_cloud.header.frame_id = camera_frame;
    out_cloud.header.stamp = ros::Time::now();
    pub.publish(out_cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetPointCloudROI::applyPassthroughFilter(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (in_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_min, x_max);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_min, y_max);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max);
    pass.filter (*cloud_filtered);

    return cloud_filtered;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "apply_pc_roi");
    ros::NodeHandle n;
    GetPointCloudROI get_pc_roi(n);
    
    return 0;
}