#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "tool_tf_listener");
	ros::NodeHandle node;

	tf::TransformListener listener;

	ros::Rate rate(10.0);

	while (node.ok()){
	    tf::StampedTransform transform;
	    try{
	    	ros::Time now = ros::Time::now();
			listener.waitForTransform("link_1", "tool_link", now, ros::Duration(3.0));
			listener.lookupTransform("link_1","tool_tip_link", ros::Time(0), transform);

			std::cout <<"tool tip link x" << transform.getOrigin().x()<<std::endl;
			std::cout <<"tool tip link y" << transform.getOrigin().y()<<std::endl;
			std::cout <<"tool tip link z" << transform.getOrigin().z()<<std::endl;
	    }
	    catch (tf::TransformException ex){
	      ROS_ERROR("%s",ex.what()); 
	      ros::Duration(1.0).sleep();
	    }
	  return 0;
	}
}