#include <iostream>
#include <vector>

#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud

#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/filters/extract_indices.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
 
main (int argc, char **argv)
{
  ros::init (argc, argv, "pcd_to_rviz");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
  
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);

  sensor_msgs::PointCloud2 output;
  pcl::io::loadPCDFile ("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/real_pc.pcd", *cloud); //Modify the path of your pcd file

  pcl::toROSMsg(*cloud, output);

  //Convert the cloud to ROS message
  output.header.frame_id = "pcd";
  output.header.stamp = ros::Time::now();
  
  while (ros::ok())
  {
    pcl_pub.publish(output);
    // pc_seg_pub.publish(seg_output);
    ros::spinOnce();
  }
  return 0;
}
