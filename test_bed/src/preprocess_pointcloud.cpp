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

void PCprocess::read_pcd(){
  std::cout<<"test read" <<std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/test_file.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_file.pcd with the following fields: "
            << std::endl;
  // for (const auto& point: *cloud)
  //   std::cout << "    " << point.x
  //             << " "    << point.y
  //             << " "    << point.z << std::endl;
  }
}

void PCprocess::cloud_cd(const sensor_msgs::PointCloud2 msg){
    pcl::PCLPointCloud2 pcl_pc; //pcl point cloud
    pcl_conversions::toPCL(msg, pcl_pc); // sensor msg to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//pointcloud2
    pcl::fromPCLPointCloud2(pcl_pc,*temp_cloud); //pcl to pointcloud2
    // sensor_msgs::PointCloud2 ros_output;
    // pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
    // pcl_conversions::fromPCL(pcl_pc, ros_output);
    if(flag_save ==0){
      std::cout << "saveing.."<< std::endl;
      pcl::io::savePCDFile ("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/origianl_pc.pcd", *temp_cloud);
      flag_save = 1;
      std::cout << "saved" << std::endl;
    }
    //do_passthrough(*temp_cloud, ptfilter);
    pcl::PointCloud<pcl::PointXYZ> pc_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> ptfilter;

    *ptr_filtered = *temp_cloud;

    ptfilter.setInputCloud(ptr_filtered);
    ptfilter.setFilterFieldName("z"); 
    ptfilter.setFilterLimits(0.51, 0.64); //min. max
    ptfilter.setFilterLimitsNegative(true); //option 
    ptfilter.filter(*ptr_filtered);
    pc_filtered = *ptr_filtered;
    std::cout << "filtered saving..." << std::endl;
    pcl::io::savePCDFile ("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/passfilter_pc.pcd", pc_filtered);
    std::cout << "filtered saved" << std::endl;

}

// void PCprocess::do_passthrough(const pcl::PointCloud<pcl::PointXYZ>& src, pcl::PointCloud<pcl::PointXYZ>& dst){
//   pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PassThrough<pcl::PointXYZ> ptfilter;

//   *ptr_filtered = src;

//   ptfilter.setInputCloud(ptr_filtered);
//   ptfilter.setFilterFieldName("z"); 
//   ptfilter.setFilterLimits(0.001, 0.1); //min. max
//   ptfilter.setFilterLimitsNegative(false); //option 
//   ptfilter.filter(*ptr_filtered);
//   dst = *ptr_filtered;
//   // pass.setInputCloud (cloud);
//   // pass.setFilterFieldName ("z");
//   // pass.setFilterLimits (0.03, 0.04);
//   // pass.filter (*cloud_filtered);

// }

int main(int argc, char **argv){
  std::cout << "start!"<< std::endl;
  ros::init (argc, argv, "pc_process");
  PCprocess wp_object;
  wp_object.read_pcd();
  ros::spin ();
  return 0;
}