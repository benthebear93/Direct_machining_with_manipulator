#include "preprocess_pointcloud.h"
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_t;

PCprocess::PCprocess()
{
  mbflag_save = 0;
  mbflag_cluster_save = 0;
  std::cout << "point cloud process start" << std::endl;
  sub_ = n_.subscribe("/camera/depth/color/points", 1 , &PCprocess::Cloudcb, this);
  //boundary_pub_= n_.advertise<tx90_path_planner::boundary>("boundary", 100);
  pc_pub_ = n_.advertise<sensor_msgs::PointCloud2> ("pc_output", 1);
  pc_segpub_ = n_.advertise<sensor_msgs::PointCloud2> ("seg_out", 1);
}

PCprocess::~PCprocess()
{

}

void PCprocess::Segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud)
{
  /* 
  segment filtered point cloud to extract specimen from scene. 
  it will save segmented pointcloud with given numbers.
  */
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
  pcl::PointCloud<pcl::PointXYZRGB> TotalCloud;
 int currentClusterNum = 1;
 if(mbflag_cluster_save == 0)
 {
    for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
    {
        // add all its points to a new cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
            cluster->points.push_back(in_cloud->points[*point]);

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        // and save it to disk.
        if (cluster->points.size() <= 0)
            break;
        std::string fileName = filepath +"/pcd_data/new_cluster" + boost::to_string(currentClusterNum) + ".pcd";
        pcl::io::savePCDFileASCII(fileName, *cluster);
        currentClusterNum++;
    }
    mbflag_cluster_save = 1;
  }
  cluster_cloud = reg.getColoredCloud();
  pcl::io::savePCDFile<pcl::PointXYZRGB>(filepath + "/pcd_data/region_growing_rgb_result.pcd", *cluster_cloud);
}

void PCprocess::Cloudcb(const sensor_msgs::PointCloud2 msg){ //std_msgs::Int32 msg

  pcl::PointCloud<pcl::PointXYZRGB> pc_transformed;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

  if(mbflag_save ==0)
  {
    pcl::PCLPointCloud2 pcl_pc; //pcl point cloud
    pcl_conversions::toPCL(msg, pcl_pc); // sensor msg to pcl
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//pointcloud2
    pcl::fromPCLPointCloud2(pcl_pc,*temp_cloud); //pcl to pointcloud2
    ROS_INFO("save original pc");
    pcl::io::savePCDFile (filepath + "/pcd_data/actual_pc_v4.pcd", *temp_cloud);
    Eigen::Matrix4f trans;

    trans<< 0,   -1,  0, 0.610,
            -1,   0,  0, 0.077,
            0,   0,  -1, 0.765,
            0,   0,  0,     1;
    pcl::transformPointCloud(*temp_cloud , *ptr_transformed, trans);   
    pc_transformed = *ptr_transformed;
    ROS_INFO("save tf changed pc");
    pcl::io::savePCDFile (filepath + "/pcd_data/changed_pc_v4.pcd", pc_transformed);
    mbflag_save = 1;
  }
  else
  {
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (filepath + "/pcd_data/changed_pc_v4.pcd", *ptr_transformed);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*ptr_transformed, output);

    output.header.frame_id = "world";
    output.header.stamp = ros::Time::now();
    pc_pub_.publish(output);

    //pcl::PointCloud<pcl::PointXYZRGB> pc_filtered;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> ptfilter;

    *ptr_filtered = *ptr_transformed;

    ptfilter.setInputCloud(ptr_filtered);
    ptfilter.setFilterFieldName("z"); 
    ptfilter.setFilterLimits(0.141, 0.25);  // min. max
    ptfilter.setFilterLimitsNegative(false); // option 
    ptfilter.filter(*ptr_filtered);

    //pc_filtered = *ptr_filtered;

    PCprocess::Segmentation(ptr_filtered);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (filepath + "/pcd_data/new_cluster4.pcd", *segmented_pc);
    //PCprocess::ExtractBorder(segmented_pc);
  }
}

int main(int argc, char **argv)
{
  std::cout << "start!"<< std::endl;
  ros::init (argc, argv, "pc_process");
  PCprocess wp_object;
  // wp_object.read_pcd();
  ros::spin ();
  return 0;
}