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
#include <pcl/common/transforms.h>

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
  pcl::io::loadPCDFile ("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/cluster10.pcd", *cloud); //Modify the path of your pcd file

  pcl::PointCloud<pcl::PointXYZRGB> pc_transformed;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

  Eigen::Matrix4f trans;
  trans<< 0,   1,  0, 0.645298,
          1,   0,  0, 0.0,
          0,   0,  -1,  1.43868,
          0,   0,  0,     1;
  pcl::transformPointCloud(*cloud, *ptr_transformed, trans);

  pc_transformed = *ptr_transformed;
  std::cout <<"transforms" << std::endl;
  pcl::io::savePCDFile ("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/tf_test.pcd", pc_transformed);

  
  pcl::toROSMsg(*ptr_transformed, output);

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
