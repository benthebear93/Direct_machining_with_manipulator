#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "tool_tf_listener");
	ros::NodeHandle nh;

	tf::TransformListener listener;

	ros::Rate rate(10.0);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	uint32_t shape = visualization_msgs::Marker::ARROW;
	std::cout <<"before while" <<std::endl;
	while (nh.ok()){
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/world";
		marker.header.stamp = ros::Time::now();
	    marker.ns = "basic_shapes";
	    marker.id = 0;

	    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	    marker.type = shape;

	    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	    marker.action = visualization_msgs::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = 0.231156;
	    marker.pose.position.y = -0.049475;
	    marker.pose.position.z = 1.33092;
	    marker.pose.orientation.x = -0.707079;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.707134;
	    marker.pose.orientation.w = 0.0;

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = 0.1;
	    marker.scale.y = 0.01;
	    marker.scale.z = 0.01;

	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 0.0f;
	    marker.color.g = 1.0f;
	    marker.color.b = 0.0f;
	    marker.color.a = 1.0;
	    std::cout <<"setting marker" <<std::endl;
	    marker.lifetime = ros::Duration();
	    marker_pub.publish(marker);
	    // marker for workpiece point
	    std::cout <<"after marker" <<std::endl;
	    tf::StampedTransform tool_tf;
	    tf::StampedTransform camera_tf;
	    tf::Quaternion tool_q;
	    tf::Quaternion cam_q;

	    tf::Transform wp_tf;
	    wp_tf.setOrigin(tf::Vector3(0.029, 0.082, 0.449));
	    tf::Quaternion wp_q;
	    wp_q.setRPY(0, 0, 0);
	    wp_tf.setRotation(wp_q);
	    std::cout <<"before tf?" <<std::endl;
	    try{
	    	ros::Time now = ros::Time::now();
			listener.waitForTransform("world", "tool_link", now, ros::Duration(3.0));
			listener.lookupTransform("world","tool_tip_link", ros::Time(0), tool_tf);

			std::cout <<"tool tip link x : " << tool_tf.getOrigin().x();
			std::cout <<" y : " << tool_tf.getOrigin().y();
			std::cout <<" z : " << tool_tf.getOrigin().z()<<std::endl;
			tool_q = tool_tf.getRotation();
			std::cout <<" q_x: " << tool_q.x()<<std::endl;
			std::cout <<" q_y: " << tool_q.y()<<std::endl;
			std::cout <<" q_z: " << tool_q.z()<<std::endl;
			std::cout <<" q_w: " << tool_q.w()<<std::endl;

			listener.waitForTransform("world", "camera_link", now, ros::Duration(3.0));
			listener.lookupTransform("world","camera_link", ros::Time(0), camera_tf);
			std::cout <<"camera x : " << camera_tf.getOrigin().x();
			std::cout <<" y : " << camera_tf.getOrigin().y();
			std::cout <<" z : " << camera_tf.getOrigin().z()<<std::endl;
			cam_q = camera_tf.getRotation();
			std::cout <<" q_x : " << cam_q.x()<<std::endl;
			std::cout <<" q_y : " << cam_q.y()<<std::endl;
			std::cout <<" q_z : " << cam_q.z()<<std::endl;
			std::cout <<" q_w : " << cam_q.w()<<std::endl;
			std::cout << "========================" << std::endl;
			// get tool tf and cam tf
			tf::Transform C = camera_tf * wp_tf;
			std::cout <<"w_camera x : " << C.getOrigin().x();
			std::cout <<" y : " << C.getOrigin().y();
			std::cout <<" z : " << C.getOrigin().z()<<std::endl;
			cam_q = C.getRotation();
			std::cout <<" q_x : " << cam_q.x()<<std::endl;
			std::cout <<" q_y : " << cam_q.y()<<std::endl;
			std::cout <<" q_z : " << cam_q.z()<<std::endl;
			std::cout <<" q_w : " << cam_q.w()<<std::endl;
			std::cout << "========================" << std::endl;
	    }
	    catch (tf::TransformException ex){
	      ROS_ERROR("%s",ex.what()); 
	      ros::Duration(1.0).sleep();
	    }
	}
  return 0;
}