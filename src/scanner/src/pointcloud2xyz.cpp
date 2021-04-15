#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <detection_msgs/BBox2D.h>
#include <detection_msgs/Detection2D.h>
#include <detection_msgs/Det3D.h>
#include <detection_msgs/Det3DArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher pub_xyz;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_out (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 save_cloud;

void cloud_cb (const sensor_msgs::PointCloud2 cloud_in)
{
    pcl::fromROSMsg (cloud_in, *cloud_xyz_out);
    save_cloud = cloud_in;
}

void box_cb (const detection_msgs::Detection2D input)
{
    detection_msgs::Det3DArray position_array;
    detection_msgs::Det3D item_position;
    bool flag = true;

    for(int i=0; i<input.boxes.size(); i++)
    {
        int x = input.boxes[i].center.x;
        int y = input.boxes[i].center.y;
        int index = x + y * cloud_xyz_out->width;
        std::cout<<"x = "<<x<<std::endl;
        std::cout<<"y = "<<y<<std::endl;
        std::cout<<"cloud_msg_width = "<<cloud_xyz_out->width<<std::endl;
        std::cout<<"cloud_msg_height = "<<cloud_xyz_out->height<<std::endl;
        std::cout<<i<<" cloud_point_size = "<<cloud_xyz_out->points.size()<<std::endl;
        std::cout<<"index = "<<index<<std::endl;
        
        if(isnan(cloud_xyz_out->points[index].x) && isnan(cloud_xyz_out->points[index].y) && isnan(cloud_xyz_out->points[index].z))
            flag = false;
        else
        {
            float x3Ddepth = cloud_xyz_out->points[index].x;
            float y3Ddepth = cloud_xyz_out->points[index].y;
            float z3Ddepth = cloud_xyz_out->points[index].z;
            item_position.x = z3Ddepth;
            item_position.y = -x3Ddepth;
            item_position.z = -y3Ddepth;           
            item_position.class_name = input.boxes[i].class_name;
            item_position.class_id = input.boxes[i].id;
            position_array.dets_list.push_back(item_position);
            position_array.header = input.header;
            position_array.pointcloud = save_cloud;

            // tf_broadcaster
            // static tf::TransformBroadcaster br;
            // tf::Transform transform;
            // transform.setOrigin(tf::Vector3(item_position.position.x,item_position.position.y,item_position.position.z));
            // tf::Quaternion q;
            // q.setEuler(0.0,0.0,0.0);
            // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"rs_camera_link", "item_link"));
        }
    }

    if(flag)
        pub_xyz.publish(position_array);
    else
        std::cout<<"No data receive!!!!"<<std::endl;

    for(int i=0; i<position_array.dets_list.size(); i++)
    {
        std::cout<<position_array.dets_list[i].class_name<<std::endl;
        std::cout<<"x = "<<position_array.dets_list[i].x<<std::endl;    
        std::cout<<"y = "<<position_array.dets_list[i].y<<std::endl;    
        std::cout<<"z = "<<position_array.dets_list[i].z<<std::endl;
        std::cout<<"========================================================="<<std::endl<<std::endl;     
    }      
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pointcloud2xyz");
  ros::NodeHandle nh;
  ros::Subscriber sub_cloud =nh.subscribe("/scan_pointcloud_filtered", 1, cloud_cb);
  ros::Subscriber sub_box =nh.subscribe("/DetectedImgNode/det2D_result", 1, box_cb);
  pub_xyz = nh.advertise<detection_msgs::Det3DArray> ("object_detection/item_position", 1);
  ros::spin ();
  return 0;
}
