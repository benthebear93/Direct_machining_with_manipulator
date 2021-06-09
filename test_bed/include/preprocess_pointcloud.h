//preprocess_pointcloud.h
#ifndef PREPROCESS_POINTCLOUD
#define PREPROCESS_POINTCLOUD

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Int32MultiArray.h"

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#include <typeinfo> 
#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud

#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension
class PCprocess
{
public: 
	PCprocess();
	~PCprocess();
	void cloud_cd(const sensor_msgs::PointCloud2 msg);
	// void do_passthrough(const pcl::PointCloud<pcl::PointXYZRGB>& src, pcl::PointCloud<pcl::PointXYZRGB>& dst);
	void read_pcd();
	void segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud);
	void extract_border(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud);
private:
	int argc;
	char** argv;
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	ros::Publisher pub_;
	ros::Publisher boundary_pub_x_;
	ros::Publisher boundary_pub_y_;
	bool flag_save;

};
#endif