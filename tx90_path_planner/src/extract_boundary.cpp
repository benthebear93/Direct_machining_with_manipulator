#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/range_image/range_image.h>
#include <pcl/common/common_headers.h>

#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension

#include <iostream>
#include <string>
#include "tx90_path_planner/boundary.h"

using namespace std;

int main(int argc, char **argv)
{
	ROS_INFO("Extract boundary");
	ros::init (argc, argv, "extract_boundary");
	ros::NodeHandle nh;
	ros::Publisher boundary_pub = nh.advertise<tx90_path_planner::boundary>("boundary", 100);
	ros::Rate loop_rate(100);

	string filepath = "/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_path_planner"; // basic file path

	float angular_resolution= 0.5f;
	angular_resolution = pcl::deg2rad (angular_resolution);

	bool setUnseenToMaxRange = false;
	setUnseenToMaxRange = true;
	int tmp_coordinate_frame;

	typedef pcl::PointXYZ PointType;
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

	pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>); // making point cloUd
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr; // getting address
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges; // set range

	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ()); // set sensor pose
	std::string filename = filepath + "/pcd_data/fig_cluster4.pcd";
	pcl::io::loadPCDFile (filename, point_cloud);
	scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
	                                                         point_cloud.sensor_origin_[1],
	                                                         point_cloud.sensor_origin_[2])) *
	                  Eigen::Affine3f (point_cloud.sensor_orientation_);
	std::cout<< "sensor origin: " << point_cloud.sensor_origin_[0] << point_cloud.sensor_origin_[1] << point_cloud.sensor_origin_[2] << std::endl;

	//std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";

	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 2;
	pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;   
	range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
	                               scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	range_image.integrateFarRanges (far_ranges);
	if (setUnseenToMaxRange)
	range_image.setUnseenToMaxRange ();

	// Extract borders  
	pcl::RangeImageBorderExtractor border_extractor (&range_image);
	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	border_extractor.compute (border_descriptions);
	//std::cout << border_descriptions <<std::endl;

	// Show points in 3D viewer
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
	while(ros::ok())
	{
		// custom ros msgs type init 
		tx90_path_planner::boundary bounday_array;

		for (int i =0; i<border_points.points.size(); i++)
		{
			float x = border_points.points[i].x;
			float y = border_points.points[i].y;

			bounday_array.boundary_x.push_back(x);
			bounday_array.boundary_y.push_back(y);
		}
		boundary_pub.publish(bounday_array);
	}
}