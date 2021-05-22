#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>
#define PI 3.14159265
double rad2deg(double radian);
double deg2rad(double degree);
double rad2deg(double radian)
{
    return radian*180/PI;
}
double deg2rad(double degree)
{
    return degree*PI/180;
}

void find_normal()
{
  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/cloud_cluster_0.pcd)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);    
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/pass_pc_rgb.pcd", *cloud);
  std::cout << "                 " << std::endl;
  std::cout << "finding normal......" << std::endl;
  std::cout << "                 " << std::endl;
  std::cout << "input cloud size: " << cloud->points.size() << std::endl;
  
  // 시각적 확인을 위해 색상 통일 (255,255,255)
  for (size_t i = 0; i < cloud->points.size(); ++i){
  cloud->points[i].r = 255;
  cloud->points[i].g = 255;
  cloud->points[i].b = 255;
  }
  pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/before_seg.pcd", *cloud);

  //KdTree 오브젝트 생성 
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (cloud);    //입력 

     pcl::PointXYZRGB searchPoint;
     searchPoint.x = -0.006;
     searchPoint.y = 0.098;
     searchPoint.z = 0.503;
  // pcl::PointXYZRGB searchPoint = cloud->points[3000]; 

  //기준점 좌표 출력 
  std::cout << "searchPoint :" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z  << std::endl;


  //기준점에서 가까운 순서중 K번째까지의 포인트 탐색 (K nearest neighbor search)
  int K = 1000;   // 탐색할 포인트 수 설정 
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    //시각적 확인을 위하여 색상 변경 (0,255,0)
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
    {
      cloud->points[pointIdxNKNSearch[i]].r = 0;
      cloud->points[pointIdxNKNSearch[i]].g = 0;
      cloud->points[pointIdxNKNSearch[i]].b = 255;
    }
  }

  // // 탐색된 점의 수 출력 
  std::cout << "searched points ：" << pointIdxNKNSearch.size() << std::endl;

  float curvature;
  Eigen::Vector4f plane_parameters; 
  computePointNormal(*cloud,pointIdxNKNSearch,plane_parameters,curvature); 
  std::cout << "plane param : \n"<< plane_parameters << std::endl;
  std::cout << "curvature : "<< curvature << std::endl;
  std::cout <<" "<<std::endl;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
  pcl::PointCloud<pcl::Normal> sourceNormals;
  sourceNormals.push_back(pcl::Normal(plane_parameters[0], plane_parameters[1], plane_parameters[2]));
  pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/after_seg.pcd", *cloud);

  double roll_deg  = deg2rad(31.61866); // X --> Z가 되야함 
  double pitch_deg = deg2rad(89.960435);// Y --> Y는 맞음
  double yaw_deg   = deg2rad(0);        // Z --> X가 되야함
  std::cout <<" "<<std::endl;
  tf::Quaternion normal_q;

  normal_q.setEuler(pitch_deg, yaw_deg, roll_deg); //Y X Z
  normal_q = normal_q.normalize();
  std::cout << "x :" << normal_q.x() << std::endl;
  std::cout << "y :" << normal_q.y() << std::endl;
  std::cout << "z :" << normal_q.z() << std::endl;
  std::cout << "w :" << normal_q.w() << std::endl;


}

int main (int argc, char** argv)
{
  find_normal();
  ros::init(argc, argv, "find_normal");
  ros::spin();
  return 0;
}