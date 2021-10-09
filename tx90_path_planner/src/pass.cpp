#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include <pcl/filters/passthrough.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/range_image/range_image.h>
#include <pcl/common/common_headers.h>

#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension

#include <iostream>
#include <string>
#include "tx90_path_planner/boundary.h"
using namespace std;

int main(int argc, char **argv)
{
	ROS_INFO("Extract pass");
	ros::init (argc, argv, "pass");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	string filepath = "/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_path_planner"; // basic file path
	typedef pcl::PointXYZ PointType;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::io::loadPCDFile<pcl::PointXYZRGB> (filepath + "/pcd_data/changed_pc_v4.pcd", *ptr_transformed);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> ptfilter;

    *ptr_filtered = *ptr_transformed;

    ptfilter.setInputCloud(ptr_filtered);
    ptfilter.setFilterFieldName("z"); 
    ptfilter.setFilterLimits(0.141, 0.25);  // min. max
    ptfilter.setFilterLimitsNegative(false); // option 
    ptfilter.filter(*ptr_filtered);
    pcl::io::savePCDFile (filepath + "/pcd_data/passfilter_pc.pcd", *ptr_filtered);

    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud (*ptr_filtered, *indices);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (ptr_filtered);
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
    int mbflag_cluster_save = 0;
    if(mbflag_cluster_save == 0)
    {
    for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
    {
        // add all its points to a new cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
            cluster->points.push_back(ptr_filtered->points[*point]);

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        for (size_t i = 0; i < cluster->points.size(); ++i){
            cluster->points[i].r = 255;
            cluster->points[i].g = 255;
            cluster->points[i].b = 255;
        }

        // and save it to disk.
        if (cluster->points.size() <= 0)
            break;
        std::string fileName = filepath +"/pcd_data/fig_cluster" + boost::to_string(currentClusterNum) + ".pcd";
        pcl::io::savePCDFileASCII(fileName, *cluster);
        currentClusterNum++;
    }
    mbflag_cluster_save = 1;
    }
    cluster_cloud = reg.getColoredCloud();
}