#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>
#include "std_msgs/Int32MultiArray.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <math.h>
#include <string>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#define PI 3.14159265

std::string filepath = "/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_path_planner"; // basic file path

double rad2deg(double radian)
{
    return radian*180/PI;
}
double deg2rad(double degree)
{
    return degree*PI/180;
}

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
    ros::Publisher drill_point_pose_;
    ros::Publisher pc_pub_;
    float dp_x = 0;
    float dp_y = 0;
    float dp_z = 0;
};

FindNormal::FindNormal()
{
	drill_point_pub_  = n_.advertise<std_msgs::Int32MultiArray>("drill_point", 1);
  drill_point_pose_ = n_.advertise<geometry_msgs::Pose>("drill_pose", 1); 
  pc_pub_ = n_.advertise<sensor_msgs::PointCloud2> ("rviz_pc", 1);
  
}
FindNormal::~FindNormal(){

}

void FindNormal::find_normal(){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);    
  pcl::PointCloud<pcl::PointXYZ>::Ptr scanned_cloud (new pcl::PointCloud<pcl::PointXYZ>);    
  pcl::io::loadPCDFile<pcl::PointXYZ>(filepath+"/pcd_data/final_scan2.pcd", *scanned_cloud);
  

  copyPointCloud(*scanned_cloud, *temp_cloud); //linescanner xyz to xyzrgb for visualization

  pcl::PassThrough<pcl::PointXYZRGB> ptfilter;

  ptfilter.setInputCloud(temp_cloud);
  ptfilter.setFilterFieldName("z"); 
  ptfilter.setFilterLimits(0.250, 0.300);  // min. max
  ptfilter.setFilterLimitsNegative(false); // option 
  ptfilter.filter(*temp_cloud);

  std::cout << "copying ....." << std::endl;
  std::cout << "                 " << std::endl;
  std::cout << "finding normal......" << std::endl;
  std::cout << "                 " << std::endl;
  std::cout << "input cloud size: " << scanned_cloud->points.size() << std::endl;
  
  // set rgb value as black 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (temp_cloud);
  sor.setLeafSize (0.001f, 0.001f, 0.001f);
  sor.filter (*cloud);
  std::cout << "downsampled cloud size: " << cloud->points.size() << std::endl;
  for (size_t i = 0; i < cloud->points.size(); ++i){
  cloud->points[i].r = 0;
  cloud->points[i].g = 0;
  cloud->points[i].b = 0;
  }

  //KdTree object init
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (cloud);    //입력 

  pcl::PointXYZRGB searchPoint;
  searchPoint.x = 0.732; 
  searchPoint.y = -0.040;
  searchPoint.z = 0.285;

  //print search point
  std::cout << "searchPoint :" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z  << std::endl;

  //select K nearest neighbor point
  int K = 10;   // 탐색할 포인트 수 설정 
  std::vector<int> pointIdxNKNSearch(K);
  float radius = 0.01;
  std::vector<float> pointNKNSquaredDistance(radius);
  double temp_x = 0;
  double temp_y = 0;
  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
    // colorized searched point
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
      cloud->points[pointIdxNKNSearch[i]].r = 0;
      cloud->points[pointIdxNKNSearch[i]].g = 0;
      cloud->points[pointIdxNKNSearch[i]].b = 255;
      temp_x = cloud->points[pointIdxNKNSearch[i]].x;
      temp_y = cloud->points[pointIdxNKNSearch[i]].y;
      std::cout << "temp_x: " << temp_x << "temp_y: "<<temp_y << std::endl;
    }
  }
  // number of searched point (should be same with set search point number)
  // std::cout << "searched points ：" << pointIdxNKNSearch.size() << std::endl;

  float curvature;
  Eigen::Vector4f plane_parameters; 
  computePointNormal(*cloud, pointIdxNKNSearch, plane_parameters, curvature); 
  std::cout << "plane param : \n" << plane_parameters << std::endl;
  std::cout << "curvature : " << curvature << std::endl;
  std::cout << " " <<std::endl;

  double x = plane_parameters[0];
  double y = plane_parameters[1];
  double z = plane_parameters[2];
  double w = plane_parameters[3];
  
  std::cout <<"x: "<< x<<" y: "<< y <<" z: "<<z<<std::endl;
  double pitch_val = z/x;
  double roll_deg  = atan2(y, z);
  double pitch_deg = atan2(z, x);//atan2(z, x);
  double yaw_deg   = atan2(y, x); // rad
  std::cout << "before roll : " << rad2deg(roll_deg) << " pitch : "<< rad2deg(pitch_deg) << "yaw : " << rad2deg(yaw_deg) << std::endl;
  if(pitch_deg <0){
    pitch_deg = 1.5708 + pitch_deg;
    for (int i=0; i <50; i++){
      cloud->points[500+i].x = searchPoint.x - i*x/100.0;
      cloud->points[500+i].y = searchPoint.y + i*y/100.0;
      cloud->points[500+i].z = searchPoint.z - i*z/100.0;
    }
  }
  else if(pitch_deg >1.5708){
    pitch_deg = PI - pitch_deg;
    yaw_deg = PI - yaw_deg;
  }
  else{
    roll_deg = PI-roll_deg;
    yaw_deg = -PI+yaw_deg;
    for (int i=0; i <50; i++){
      // if(i==0){
      //   temp_x = searchPoint.x + i*x/100.0;
      //   temp_y = searchPoint.y + i*y/100.0;
      //   std::cout << "temp_x: " << temp_x << "temp_y: "<<temp_y << std::endl;
      // }
      temp_x = searchPoint.x + i*x/100.0;
      temp_y = searchPoint.y + i*y/100.0;
      cloud->points[500+i].x = searchPoint.x + i*x/100.0;
      cloud->points[500+i].y = searchPoint.y + i*y/100.0;
      cloud->points[500+i].z = searchPoint.z + i*z/100.0;
    }
  }

  pcl::io::savePCDFileASCII(filepath+"/pcd_data/downsample_normal_surface.pcd", *cloud);

  std::cout << "after roll : " << rad2deg(roll_deg) << " pitch : "<< rad2deg(pitch_deg) << "yaw : " << rad2deg(yaw_deg) << std::endl;
  std::cout << "   " << std::endl;
  tf::Quaternion normal_q;  
  //else:

  std::cout << "x :" << rad2deg(roll_deg) << std::endl;
  std::cout << "y :" << rad2deg(pitch_deg) << std::endl;
  std::cout << "z :" << rad2deg(yaw_deg) << std::endl;
  std::cout << "   " << std::endl;

  normal_q.setRPY(roll_deg, pitch_deg, yaw_deg); //Y X Z 
  normal_q = normal_q.normalize();

  std::cout << "q.x :" << normal_q.x() << std::endl;
  std::cout << "q.y :" << normal_q.y() << std::endl;
  std::cout << "q.z :" << normal_q.z() << std::endl;
  std::cout << "q.w :" << normal_q.w() << std::endl;
  std::cout << "===============" << std::endl;
  geometry_msgs::Pose drill_pose;
  drill_pose.position.x = searchPoint.x;
  drill_pose.position.y = searchPoint.y;
  drill_pose.position.z = searchPoint.z;
  drill_pose.orientation.x = normal_q.x();
  drill_pose.orientation.y = normal_q.y();
  drill_pose.orientation.z = normal_q.z();
  drill_pose.orientation.w = normal_q.w();
  drill_point_pose_.publish(drill_pose);
  sensor_msgs::PointCloud2 output;
  output.header.frame_id = "world";
  output.header.stamp = ros::Time::now();
  pcl::toROSMsg(*cloud, output);
  pc_pub_.publish(output);

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "find_normal");
  FindNormal findnormal;
  while (ros::ok()) findnormal.find_normal();
  ros::spin();
}