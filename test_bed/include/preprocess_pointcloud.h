//preprocess_pointcloud.h
#ifndef PREPROCESS_POINTCLOUD
#define PREPROCESS_POINTCLOUD

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <typeinfo> 
#include <pcl/filters/passthrough.h>

class PCprocess
{
public: 
	PCprocess();
	~PCprocess();
	void cloud_cd(const sensor_msgs::PointCloud2 msg);
	// void do_passthrough(const pcl::PointCloud<pcl::PointXYZRGB>& src, pcl::PointCloud<pcl::PointXYZRGB>& dst);
	void read_pcd();
private:
	int argc;
	char** argv;
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	ros::Publisher pub_;
	bool flag_save;

};
#endif