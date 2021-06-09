#include "preprocess_pointcloud.h"
using namespace std;
//ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_t;
// typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;

PCprocess::PCprocess()
{
  flag_save = 0;
  std::cout << "point cloud process start" << std::endl;
  sub_ = n_.subscribe("/pcl_output", 1 , &PCprocess::cloud_cd, this);
  boundary_pub_x_ = n_.advertise<std_msgs::Int32MultiArray>("boundary_x", 100);
  boundary_pub_y_ = n_.advertise<std_msgs::Int32MultiArray>("boundary_y", 100);
}

PCprocess::~PCprocess(){

}
// basic class 

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
  }
}
// upload pcd

void PCprocess::segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud){
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud (*in_cloud, *indices);
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (in_cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (100);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;


  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" <<
  std::endl << "cloud that belong to the first cluster:" << std::endl;
 int currentClusterNum = 1;
  for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
  {
      // ...add all its points to a new cloud...
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
          cluster->points.push_back(in_cloud->points[*point]);
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;

      // ...and save it to disk.
      if (cluster->points.size() <= 0)
          break;
      std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
      std::string fileName = "/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/cluster" + boost::to_string(currentClusterNum) + ".pcd";
      pcl::io::savePCDFileASCII(fileName, *cluster);

      currentClusterNum++;
  }
}
//segmentation

void PCprocess::extract_border(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud)
{
  typedef pcl::PointXYZ PointType;
  float angular_resolution= 0.5f;
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  bool setUnseenToMaxRange = false;

  setUnseenToMaxRange = true;
  int tmp_coordinate_frame;
  angular_resolution = pcl::deg2rad (angular_resolution);

  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>); //making point cloUd
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr; //getting address
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges; //set range
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ()); //set sensor pose
  std::string filename = "/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/cluster10.pcd";
  pcl::io::loadPCDFile (filename, point_cloud);
  scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                             point_cloud.sensor_origin_[1],
                                                             point_cloud.sensor_origin_[2])) *
                      Eigen::Affine3f (point_cloud.sensor_orientation_);
  std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
  if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
      std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
   
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;   
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();
  
  // -------------------------
  // -----Extract borders-----
  // -------------------------
  pcl::RangeImageBorderExtractor border_extractor (&range_image);
  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
  border_extractor.compute (border_descriptions);
  std::cout << border_descriptions <<std::endl;
  
  // ----------------------------------
  // -----Show points in 3D viewer-----
  // ----------------------------------
  pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
                                            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
                                            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
  pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
                                      & veil_points = * veil_points_ptr,
                                      & shadow_points = *shadow_points_ptr;
  for (int y=0; y< (int)range_image.height; ++y)
  {
    for (int x=0; x< (int)range_image.width; ++x)
    {
      if (border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
        border_points.points.push_back (range_image[y*range_image.width + x]);
      if (border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
        veil_points.points.push_back (range_image[y*range_image.width + x]);
      if (border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
        shadow_points.points.push_back (range_image[y*range_image.width + x]);
    }
  }

  std_msgs::Int32MultiArray x_array;
  std_msgs::Int32MultiArray y_array;
  x_array.data.clear();
  y_array.data.clear();
  for (int i =0; i<border_points.points.size(); i++){
    std::cout << "x: "<<300+int(border_points.points[i].x * 1000)<< std::endl;
    int x = 300+int(border_points.points[i].x * 1000);
    std::cout << "y: "<<int(border_points.points[i].y * 1000)<< std::endl;
    int y = int(border_points.points[i].y * 1000);
    x_array.data.push_back(x);
    y_array.data.push_back(y);
  }
  boundary_pub_x_.publish(x_array);
  boundary_pub_y_.publish(y_array);
}





void PCprocess::cloud_cd(const sensor_msgs::PointCloud2 msg){
    pcl::PCLPointCloud2 pcl_pc; //pcl point cloud
    pcl_conversions::toPCL(msg, pcl_pc); // sensor msg to pcl
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//pointcloud2
    pcl::fromPCLPointCloud2(pcl_pc,*temp_cloud); //pcl to pointcloud2

    if(flag_save ==0){
      std::cout << "saveing.."<< std::endl;
      pcl::io::savePCDFile ("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/pcl_output.pcd", *temp_cloud);
      flag_save = 1;
      std::cout << "saved" << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZRGB> pc_filtered;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> ptfilter;

    *ptr_filtered = *temp_cloud;

    ptfilter.setInputCloud(ptr_filtered);
    ptfilter.setFilterFieldName("z"); 
    ptfilter.setFilterLimits(0.53, 0.600); //min. max
    ptfilter.setFilterLimitsNegative(false); //option 
    ptfilter.filter(*ptr_filtered);

    pc_filtered = *ptr_filtered;
    std::cout << "filtered saving..." << std::endl;
    pcl::io::savePCDFile ("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/pass_pcl_output.pcd", pc_filtered);
    std::cout << "filtered saved" << std::endl;

    PCprocess::segmentation(ptr_filtered);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data/cluster10.pcd", *segmented_pc);
    PCprocess::extract_border(segmented_pc);
}

int main(int argc, char **argv){
  std::cout << "start!"<< std::endl;
  ros::init (argc, argv, "pc_process");
  PCprocess wp_object;
  wp_object.read_pcd();
  ros::spin ();
  return 0;
}