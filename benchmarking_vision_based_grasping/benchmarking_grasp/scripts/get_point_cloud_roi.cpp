#include <iostream>

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
        pcl::PointCloud<pcl::PointXYZ>::Ptr applyPassthroughFilter(
            pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
        static void on_trackbar(int, void*){};

    private:
        ros::NodeHandle n;
        ros::Subscriber pt_cloud_sub;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        pcl::visualization::PCLVisualizer::Ptr viewer;
        int x_min = 45, x_max = 55, y_min = 45, y_max = 55, z_min = 0, z_max = 100;
};

GetPointCloudROI::GetPointCloudROI(ros::NodeHandle& nh): n(nh){
    std::string pc_topic;
    bool sim_mode;
    n.getParam("sim_mode", sim_mode);
    
    if(sim_mode)
        n.getParam("point_cloud_sim", pc_topic);
    else
        n.getParam("point_cloud", pc_topic);
    
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pt_cloud_sub = n.subscribe<sensor_msgs::PointCloud2>(pc_topic, 1, &GetPointCloudROI::ptCloudCallback, this);
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("Point Cloud"));
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);

    cv::Mat M(1, 400, CV_8UC3, cv::Scalar(255,255,255));
    cv::namedWindow("Adjust ROI", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("x_min", "Adjust ROI", &x_min, 100, on_trackbar, this);
    cv::createTrackbar("x_max", "Adjust ROI", &x_max, 100, on_trackbar, this);
    cv::createTrackbar("y_min", "Adjust ROI", &y_min, 100, on_trackbar, this);
    cv::createTrackbar("y_max", "Adjust ROI", &y_max, 100, on_trackbar, this);
    cv::createTrackbar("z_min", "Adjust ROI", &z_min, 100, on_trackbar, this);
    cv::createTrackbar("z_max", "Adjust ROI", &z_max, 100, on_trackbar, this);

    while (true){
        viewer->removeAllPointClouds();
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, *in_cloud);
        
        float in_x_min = x_min*0.02 - 1;
        float in_x_max = x_max*0.02 - 1;
        float in_y_min = y_min*0.02 - 1;
        float in_y_max = y_max*0.02 - 1;
        float in_z_min = z_min*0.02 - 1; 
        float in_z_max = z_max*0.02 - 1;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr passthroughCloud = applyPassthroughFilter(in_cloud, in_x_min, in_x_max, in_y_min, in_y_max, in_z_min, in_z_max);
        
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, color_handler);
        viewer->addPointCloud<pcl::PointXYZ> (passthroughCloud, "ROI");
        
        cv::imshow("Adjust ROI", M);
        int key = cv::waitKey(1) & 255; 
        if (key == 27) {
            std::cout << "crop_size: [" << in_z_min << ", " << in_y_min << ", " << in_x_min << ", " << in_z_max << ", " << in_y_max << ", " << in_x_max << "]" << std::endl;
            cv::destroyAllWindows();
            viewer->close();    
            break;
        }          
        
        viewer->spinOnce ();
        ros::spinOnce();
    }

}

void GetPointCloudROI::ptCloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_cloud){
    pcl::fromROSMsg(*in_cloud, *cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GetPointCloudROI::applyPassthroughFilter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
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
    ros::init(argc, argv, "get_pc_roi");
    ros::NodeHandle n;
    GetPointCloudROI get_pc_roi(n);

    ros::spin();
    return 0;
}