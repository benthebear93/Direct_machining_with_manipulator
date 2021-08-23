#include "ros/ros.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>


int main (int argc, char** argv)
{
  ros::init(argc, argv, "process");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/benlee/Desktop/pcd_data/0111pcd/smoot_pc.pcd", *cloud);

  // 포인트수 출력
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

  // 오브젝트 생성 
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);                //입력 
  pass.setFilterFieldName ("z");             //적용할 좌표 축 (eg. Z축)
  pass.setFilterLimits (-35, -20);          //적용할 값 (최소, 최대 값)
  //pass.setFilterLimitsNegative (true);     //적용할 값 외 
  pass.filter (*cloud_filtered);             //필터 적용 
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);  //kd tree
  pcl::PointCloud<pcl::PointNormal> mls_points; // point normal type calculated by MLS
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls; //init object
  
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud_filtered);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (mls_points);

  printf("normal estimation start");
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normalEstimation.setInputCloud(cloud_filtered);
  // For every point, use all neighbors in a radius of 3cm.
  normalEstimation.setRadiusSearch(0.03);
  // A kd-tree is a data structure that makes searches efficient. More about it later.
  // The normal estimation object will use it to find nearest neighbors.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);

  // Calculate the normals.
  normalEstimation.compute(*normals);

  // Visualize them.
  printf("viewer setting");
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
  viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud");
  // Display one normal out of 20, as a line of length 3cm.
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filtered, normals, 20, 0.03, "normals");
  // while (!viewer->wasStopped())
  // {
  //     viewer->spinOnce(100);
  //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  // }
  // 포인트수 출력
  //std::cout << "Filtered :" << cloud_filtered->width * cloud_filtered->height  << std::endl;  
  pcl::io::savePCDFile ("/home/benlee/Desktop/pcd_data/0111pcd/c_test.pcd", mls_points);
  // 저장 
  //pcl::io::savePCDFile<pcl::PointXYZ>("/home/benlee/Desktop/pcd_data/0111pcd/c_test.pcd", *cloud_filtered); //Default binary mode save

  ros::spin();
  return (0);
}