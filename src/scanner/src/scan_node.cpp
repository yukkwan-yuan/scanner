#include <iostream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;


ros::Publisher pub_combined_image;

void scan_callback(const sensor_msgs::PointCloud2 cloud_msg)
{
    // ROS PointCloud2 -> PCL Pointcloud
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_raw, *cloud_raw, indices);

    pcl::CropBox<pcl::PointXYZ> box_filter_;
    box_filter_.setMax(Eigen::Vector4f(3.0, 1.5, 1.0, 1.0));  //3.0, 1.0, 3.0, 1.0
    box_filter_.setMin(Eigen::Vector4f(-3.0, -1.5, 0.1, 1.0));  //-3.0, -1.2, 0, 1.0
    box_filter_.setKeepOrganized(false);
    box_filter_.setNegative(false);
    box_filter_.setInputCloud(cloud_raw);
    box_filter_.filter(*cloud_raw);
    //cout << cloud_raw->points.size()<< endl;

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    PointCloudXYZPtr cloud_filtered(new PointCloudXYZ);
    sor.setInputCloud (cloud_raw);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);
    //cout << cloud_filtered->points.size()<< endl;

    // Remove outlier
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    PointCloudXYZPtr cloud_out(new PointCloudXYZ);
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(0.2);
    outrem.setMinNeighborsInRadius(30);
    outrem.filter(*(cloud_out));

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_out, output);
    pub_combined_image.publish(output);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "scan_filter_node");
    ros::NodeHandle nh;
    // ROS related
    pub_combined_image = nh.advertise<sensor_msgs::PointCloud2>("/scan_pointcloud_filtered", 1);
    ros::Subscriber scan_sub = nh.subscribe("/camera1/depth_registered/points", 1, scan_callback);

    ros::spin();
    return 0;
}