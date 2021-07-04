#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include "geometry_msgs/Pose.h"
#include <iostream>


class TFlisten
{
public:
	TFlisten();
	~TFlisten();
	void drill_point_callback(const geometry_msgs::Pose msg);
private:
	int argc;
	char** argv;
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	ros::Publisher marker_pub_;
	tf::TransformListener listener_;
	float dp_x = 0;
	float dp_y = 0;
	float dp_z = 0;
	float w2c_x,w2c_y,w2c_z;
	const float norm_const = 25;
};

TFlisten::TFlisten()
{
	sub_ = n_.subscribe("/drill_point", 1, &TFlisten::drill_point_callback, this);
	marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}
TFlisten::~TFlisten(){

}

void TFlisten::drill_point_callback(const geometry_msgs::Pose msg)
{
	std::cout <<"                     " << std::endl;
	std::cout <<"listening drill point" << std::endl;
	std::cout <<"                     " << std::endl;
	dp_x = msg.position.x;
	dp_y = msg.position.y;
	dp_z = msg.position.z;
	//std::cout << dp_x << dp_y << dp_z <<std::endl;
	uint32_t shape = visualization_msgs::Marker::ARROW;
	tf::StampedTransform tool_tf; 
    tf::StampedTransform camera_tf;
    //tool and cam tf 
    tf::Quaternion tool_q;
    tf::Quaternion cam_q;
    //tool and cam quat 
    tf::Transform wp_tf;
   	tf::Quaternion wp_q;
   	//work piece tf & quat
    wp_q.setRPY(0, 0, 0);
    wp_tf.setRotation(wp_q);

    try{
    	ros::Time now = ros::Time::now();
		listener_.waitForTransform("world", "tool_tip_link", now, ros::Duration(3.0));
		listener_.lookupTransform("world","tool_tip_link", ros::Time(0), tool_tf);

		// std::cout <<"tool tip link x : " << tool_tf.getOrigin().x();
		// std::cout <<" y : " << tool_tf.getOrigin().y();
		// std::cout <<" z : " << tool_tf.getOrigin().z()<<std::endl;

		tool_q = tool_tf.getRotation();
		// std::cout <<" q_x: " << tool_q.x()<<std::endl;
		// std::cout <<" q_y: " << tool_q.y()<<std::endl;
		// std::cout <<" q_z: " << tool_q.z()<<std::endl;
		// std::cout <<" q_w: " << tool_q.w()<<std::endl;

		listener_.waitForTransform("world", "camera_depth_frame", now, ros::Duration(3.0));
		listener_.lookupTransform("world","camera_depth_frame", ros::Time(0), camera_tf);
		std::cout <<"camera x : " << camera_tf.getOrigin().x();
		std::cout <<" y : " << camera_tf.getOrigin().y();
		std::cout <<" z : " << camera_tf.getOrigin().z()<<std::endl;
		w2c_x = camera_tf.getOrigin().x();
		w2c_y = camera_tf.getOrigin().y();
		w2c_z = camera_tf.getOrigin().z();
		
		cam_q = camera_tf.getRotation();
		std::cout <<" q_x : " << cam_q.x()<<std::endl;
		std::cout <<" q_y : " << cam_q.y()<<std::endl;
		std::cout <<" q_z : " << cam_q.z()<<std::endl;
		std::cout <<" q_w : " << cam_q.w()<<std::endl;
		std::cout << "========================" << std::endl;
		// get tool tf and cam tf
    }
    catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what()); 
		ros::Duration(1.0).sleep();
    }
	visualization_msgs::Marker marker;
	visualization_msgs::Marker normal_v;

	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = w2c_x + 0.104; //y = 0.094
    marker.pose.position.y = w2c_y + 0.062; //x = 0.059
    marker.pose.position.z = w2c_z - 0.514; //z = 0.512
    std::cout<<"drill_p x : " << marker.pose.position.x << " y : " <<marker.pose.position.y << 
    " z : " << marker.pose.position.z <<std::endl;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    normal_v.header.frame_id = "/world";
	normal_v.header.stamp = ros::Time::now();
    normal_v.ns = "basic_shapes";
    normal_v.id = 1;
    normal_v.type = visualization_msgs::Marker::SPHERE;
    normal_v.action = visualization_msgs::Marker::ADD;
    normal_v.pose.position.x = w2c_x + 0.104  - 0.00078/norm_const; //y = 0.094
    normal_v.pose.position.y = w2c_y + 0.062  + 0.65747/norm_const; //x = 0.059
    normal_v.pose.position.z = w2c_z - 0.514  + 0.75348/norm_const; //z = 0.512
    std::cout<<"normal_p x : " << normal_v.pose.position.x << " y : " <<normal_v.pose.position.y << 
    " z : " << normal_v.pose.position.z <<std::endl;
    normal_v.pose.orientation.x = 0.0;
    normal_v.pose.orientation.y = 0.0;
    normal_v.pose.orientation.z = 0.0;
    normal_v.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    normal_v.scale.x = 0.01;
    normal_v.scale.y = 0.01;
    normal_v.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    normal_v.color.r = 1.0f;
    normal_v.color.g = 0.0f;
    normal_v.color.b = 0.0f;
    normal_v.color.a = 1.0;
    // std::cout <<"setting marker" <<std::endl;
    normal_v.lifetime = ros::Duration();
    marker_pub_.publish(normal_v);
    marker_pub_.publish(marker);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "tool_tf_listener");
	TFlisten tfmarker;
	//tfmarker.drill_point_marker();
	ros::spin();
	return 0;
}