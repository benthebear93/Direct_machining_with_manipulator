#include "preprocess_pointcloud.h"
using namespace std;
//ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_t;
// typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;

PCprocess::PCprocess()
{
  flag_save = 0;
  std::cout << "point cloud process start" << std::endl;
  sub_ = n_.subscribe("/camera/depth_registered/points", 1 , &PCprocess::cloud_cd, this);
}
PCprocess::~PCprocess(){

}
void PCprocess::cloud_cd(const sensor_msgs::PointCloud2 msg){
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
}

void PCprocess::do_passthrough(const pcl::PointCloud<pcl::PointXYZRGB>& src, pcl::PointCloud<pcl::PointXYZRGB>& dst){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PassThrough<pcl::PointXYZRGB> ptfilter;

  *ptr_filtered = src;

  ptfilter.setInputCloud(ptr_filtered);
  ptfilter.setFilterFieldName("z"); 
  ptfilter.setFilterLimits(0.001, 0.1); //min. max
  ptfilter.setFilterLimitsNegative(false); //option 
  ptfilter.filter(*ptr_filtered);
  dst = *ptr_filtered;
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.03, 0.04);
  // pass.filter (*cloud_filtered);

}

int main(int argc, char **argv){
  std::cout << "start!"<< std::endl;
  ros::init (argc, argv, "pc_process");
  PCprocess wp_object;
  ros::spin ();
  return 0;
}