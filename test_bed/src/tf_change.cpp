/*
author Haegu Lee

change tf of ar_marker_16 from cam to base_link
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
int main(int argc, char** argv){

	ros::init(argc, argv, "tool_tf_listener");
	ros::NodeHandle node;

	tf::TransformListener listener;

	ros::Rate rate(10.0);

	while (node.ok()){
	    tf::StampedTransform btoc; //base to cam
	    tf::StampedTransform ctom; // cam to marker
	    try{
	    	ros::Time now = ros::Time::now();
			listener.waitForTransform("base_link", "tool0", now, ros::Duration(0.1));
			listener.lookupTransform("base_link","tool0", ros::Time(0), btoc);
			double xyz[3] ={0.0,};
    		double quat[4] = {0.0, };
    		xyz[0] = btoc.getOrigin().x();
			xyz[1] = btoc.getOrigin().y();
			xyz[2] = btoc.getOrigin().z();

			quat[0] = btoc.getRotation().x();
			quat[1] = btoc.getRotation().y();
			quat[2] = btoc.getRotation().z();
			quat[3] = btoc.getRotation().w();

			// listener.waitForTransform("camera_depth_frame", "ar_marker_16", now, ros::Duration(3.0));
			// listener.lookupTransform("camera_depth_frame","ar_marker_16", ros::Time(0), ctom);

			// double ar_xyz[3] ={0.0,};
   //  		double ar_quat[4] = {0.0, };
   //  		ar_xyz[0] = ctom.getOrigin().x();
			// ar_xyz[1] = ctom.getOrigin().y();
			// ar_xyz[2] = ctom.getOrigin().z();

			// ar_quat[0] = ctom.getRotation().x();
			// ar_quat[1] = ctom.getRotation().y();
			// ar_quat[2] = ctom.getRotation().z();
			// ar_quat[3] = ctom.getRotation().w();	

			std::cout <<"x: " << xyz[0]<<"y: " << xyz[1]<<"z: " << xyz[2]<<std::endl;
			// std::cout <<"    " <<std::endl;
			// std::cout <<"ar x: " << ar_xyz[0]<<std::endl;
			// std::cout <<"ar y: " << ar_xyz[1]<<std::endl;
			// std::cout <<"ar z: " << ar_xyz[2]<<std::endl;
			// std::cout <<"    " <<std::endl;
			// static tf::TransformBroadcaster br;
			// tf::Transform marker;
			// double chg_xyz[3] ={0.0,};
			// chg_xyz[0] = ar_xyz[2] + xyz[0];
			// chg_xyz[1] = ar_xyz[1] + xyz[1];
			// chg_xyz[2] = xyz[2] - ar_xyz[0];

			// marker.setOrigin( tf::Vector3(chg_xyz[0], chg_xyz[1], chg_xyz[2]) );
			// tf::Quaternion q;
			// q.setX(ar_quat[0]);
			// q.setY(ar_quat[1]);
			// q.setZ(ar_quat[2]);
			// q.setW(ar_quat[3]);
			// marker.setRotation(q);
			// br.sendTransform(tf::StampedTransform(marker, ros::Time::now(), "base_link", "real_ar_marker_16"));
			// std::cout <<"changed x: " << marker.getOrigin().x()<<std::endl;
			// std::cout <<"changed y: " << marker.getOrigin().y()<<std::endl;
			// std::cout <<"changed z: " << marker.getOrigin().z()<<std::endl;
			// std::cout <<"    " <<std::endl;
	    }
	    catch (tf::TransformException ex){
	      ROS_ERROR("%s",ex.what()); 
	      ros::Duration(1.0).sleep();
	    }
	}
}
