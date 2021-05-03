#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <typeinfo> 
//ros::Publisher pub;
bool flag_save =0;
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_t;
// typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;

void cloud_cd(const sensor_msgs::PointCloud2 msg){
    pcl::PCLPointCloud2 pcl_pc; //pcl point cloud
    pcl_conversions::toPCL(msg, pcl_pc); // sensor msg to pcl


    // sensor_msgs::PointCloud2 ros_output;
    // pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
    // pcl_conversions::fromPCL(pcl_pc, ros_output);
    if(flag_save ==0){
      std::cout << "saveing.."<< std::endl;
      pcl::io::savePCDFile ("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/check1.pcd", pcl_pc);
      flag_save = 1;
      std::cout << "saved" << std::endl;
    }
    //pub.publish(ros_output);
}

int main(int argc, char **argv){
  std::cout << "start!"<< std::endl;
// Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cd);
  //std::cout << "saved!"<< std::endl;
  //pub = nh.advertise<sensor_msgs::PointCloud2>("/filter_pc", 1);
  // Spin
  ros::spin ();
}