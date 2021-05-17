#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <math.h>

void find_normal()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile ("/home/benlee/Desktop/git/point_cloud_practice/normal_estimation_using_integral_image/table_scene_mug_stereo_textured.pcd", *cloud);
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  	ne.setSearchMethod (tree);

  	// Output datasets
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  	ne.setRadiusSearch (0.03);

}

int main (int argc, char** argv)
{
  find_normal();
  std::cout <<" running " << std::endl;
  ros::init(argc, argv, "normal_visualize");
  ros::spin();
  return 0;
}