#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include "std_msgs/Int32MultiArray.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <math.h>
#include <string>
#define PI 3.14159265

std::string filepath = "/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_path_planner"; // basic file path
double rad2deg(double radian);
double deg2rad(double degree);

class FindNormal
{
  public:
    FindNormal();
    ~FindNormal();
    void find_normal();
  private:
    int argc;
    char** argv;
    ros::NodeHandle n_;
    ros::Publisher drill_point_pub_;
    float dp_x = 0;
    float dp_y = 0;
    float dp_z = 0;
};

FindNormal::FindNormal()
{
	drill_point_pub_ = n_.advertise<std_msgs::Int32MultiArray>("drill_point", 1);
}
FindNormal::~FindNormal(){

}

double rad2deg(double radian)
{
    return radian*180/PI;
}
double deg2rad(double degree)
{
    return degree*PI/180;
}

void FindNormal::find_normal()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);    
  pcl::PointCloud<pcl::PointXYZ>::Ptr scanned_cloud (new pcl::PointCloud<pcl::PointXYZ>);    
  pcl::io::loadPCDFile<pcl::PointXYZ>(filepath+"/pcd_data/final_scan2.pcd", *scanned_cloud);

  copyPointCloud(*scanned_cloud, *cloud); //linescanner xyz to xyzrgb for visualization
  std::cout << "copying ....." << std::endl;
  std::cout << "                 " << std::endl;
  std::cout << "finding normal......" << std::endl;
  std::cout << "                 " << std::endl;
  std::cout << "input cloud size: " << scanned_cloud->points.size() << std::endl;
  
  // set rgb value as black 
  for (size_t i = 0; i < cloud->points.size(); ++i){
  cloud->points[i].r = 0;
  cloud->points[i].g = 0;
  cloud->points[i].b = 0;
  }

  //KdTree object init
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (cloud);    //입력 

  pcl::PointXYZRGB searchPoint;
  searchPoint.x = 0.774; 
  searchPoint.y = -0.0038;
  searchPoint.z = 0.288;

  std_msgs::Int32MultiArray drill_p;
  drill_p.data.push_back(searchPoint.x*1000);
  drill_p.data.push_back(searchPoint.y*1000);
  drill_p.data.push_back(searchPoint.z*1000);
  drill_point_pub_.publish(drill_p);
  //print search point
  std::cout << "searchPoint :" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z  << std::endl;


  //select K nearest neighbor point
  int K = 1000;   // 탐색할 포인트 수 설정 
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    // colorized searched point
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
    {
      cloud->points[pointIdxNKNSearch[i]].r = 0;
      cloud->points[pointIdxNKNSearch[i]].g = 0;
      cloud->points[pointIdxNKNSearch[i]].b = 255;
    }
  }
  // number of searched point (should be same with set search point number)
  std::cout << "searched points ：" << pointIdxNKNSearch.size() << std::endl;

  float curvature;
  Eigen::Vector4f plane_parameters; 
  computePointNormal(*cloud, pointIdxNKNSearch, plane_parameters, curvature); 
  std::cout << "plane param : \n" << plane_parameters << std::endl;
  std::cout << "curvature : " << curvature << std::endl;
  std::cout << " " <<std::endl;

  double x = plane_parameters[0];
  double y = plane_parameters[1];
  double z = plane_parameters[2];
  
  double roll_deg  = atan2(y, z);
  double pitch_deg = atan2(z, x);
  double yaw_deg   = atan2(y, x); // rad

  // little modification for orientation from normal vector
  // roll_deg  = -1.5708-roll_deg;  // X
  // pitch_deg = 1.5708/2+pitch_deg;  // Y
  // yaw_deg   = -yaw_deg; // Z

  std::cout << "x :" << rad2deg(roll_deg) << std::endl;
  std::cout << "y :" << rad2deg(pitch_deg) << std::endl;
  std::cout << "z :" << rad2deg(yaw_deg) << std::endl;
  std::cout << "   " << std::endl;

  tf::Quaternion normal_q;  
  normal_q.setRPY(roll_deg, pitch_deg, yaw_deg); //Y X Z 
  normal_q = normal_q.normalize();
  std::cout << "x :" << normal_q.x() << std::endl;
  std::cout << "y :" << normal_q.y() << std::endl;
  std::cout << "z :" << normal_q.z() << std::endl;
  std::cout << "w :" << normal_q.w() << std::endl;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "find_normal");
  FindNormal findnormal;
  while (ros::ok()) findnormal.find_normal();
  ros::spin();
}