#include <ros/ros.h>
// ROS Trajectory action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with action sever
#include <actionlib/client/simple_action_client.h>

#include <descartes_moveit/ikfast_moveit_state_adapter.h>
// descartes trajectory type
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Include planner 
#include <descartes_planner/dense_planner.h>
#include <descartes_utilities/ros_conversions.h>
// making path with eigen convention
#include <eigen_conversions/eigen_msg.h>
// visualize path with marker
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>

//Some tricks from boost
#include <boost/algorithm/string.hpp>
//custom msg
#include <tx90_moveit_client/scan_path.h>

//Definitions for the path visualization
#define   AXIS_LINE_WIDTH 0.0025
#define   AXIS_LINE_LENGHT 0.01
#define   WORLD_FRAME "base_link"

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

ros::Publisher marker_publisher_;
ros::Publisher traj_pub_;
ros::Subscriber path_sub_;
std::vector<descartes_core::TrajectoryPtPtr> makePath();


EigenSTL::vector_Isometry3d HometoStart(EigenSTL::vector_Isometry3d points)
{
	Eigen::Isometry3d poseStart = points[0];

	double xStart = poseStart.translation().x();
	double yStart = poseStart.translation().y();
	double zStart = poseStart.translation().z();

	double xStep = (xStart - 0.30)/10;
	double yStep = (yStart)/10;
	double zStep = (0.40 - zStart)/10;

	Eigen::Isometry3d pose;
	for(int i=0; i<11; i++)
	{
	pose = Eigen::Translation3d(0.30 + xStep * (10 - i), yStep * (10 - i), 0.40 - zStep * (10 - i));
	points.insert(points.begin(), pose);
	}
	return points;
}

//Publish the path visualization to rviz
void publishPosesMarkers(const EigenSTL::vector_Isometry3d& poses) // vector of pose as input
{
  // creating rviz markers
  visualization_msgs::Marker z_axes, y_axes, x_axes, line;
  visualization_msgs::MarkerArray markers_msg;

  z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
  z_axes.ns = y_axes.ns = x_axes.ns = "axes";
  z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
  z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
  z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = WORLD_FRAME;
  z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = AXIS_LINE_WIDTH;

  // z properties
  z_axes.id = 0;
  z_axes.color.r = 0;
  z_axes.color.g = 0;
  z_axes.color.b = 1;
  z_axes.color.a = 1;

  // y properties
  y_axes.id = 1;
  y_axes.color.r = 0;
  y_axes.color.g = 1;
  y_axes.color.b = 0;
  y_axes.color.a = 1;

  // x properties
  x_axes.id = 2;
  x_axes.color.r = 1;
  x_axes.color.g = 0;
  x_axes.color.b = 0;
  x_axes.color.a = 1;

  // line properties
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.ns = "line";
  line.action = visualization_msgs::Marker::ADD;
  line.lifetime = ros::Duration(0);
  line.header.frame_id = WORLD_FRAME;
  line.scale.x = AXIS_LINE_WIDTH;
  line.id = 0;
  line.color.r = 1;
  line.color.g = 1;
  line.color.b = 0;
  line.color.a = 1;

  // creating axes markers
  z_axes.points.reserve(2*poses.size()); // twice of pose size? 
  y_axes.points.reserve(2*poses.size());
  x_axes.points.reserve(2*poses.size());
  line.points.reserve(poses.size());
  geometry_msgs::Point p_start,p_end;
  double distance = 0;
  Eigen::Isometry3d prev = poses[0];
  for(unsigned int i = 0; i < poses.size(); i++)
  {
    const Eigen::Isometry3d& pose = poses[i];
    distance = (pose.translation() - prev.translation()).norm();
    //std::cout << "dis : " << distance << std::endl;

    tf::pointEigenToMsg(pose.translation(),p_start);

    if(distance > 0.01)
    {
      Eigen::Isometry3d moved_along_x = pose * Eigen::Translation3d(AXIS_LINE_LENGHT,0,0);
      tf::pointEigenToMsg(moved_along_x.translation(),p_end);
      x_axes.points.push_back(p_start);
      x_axes.points.push_back(p_end);

      Eigen::Isometry3d moved_along_y = pose * Eigen::Translation3d(0,AXIS_LINE_LENGHT,0);
      tf::pointEigenToMsg(moved_along_y.translation(),p_end);
      y_axes.points.push_back(p_start);
      y_axes.points.push_back(p_end);

      Eigen::Isometry3d moved_along_z = pose * Eigen::Translation3d(0,0,AXIS_LINE_LENGHT);
      tf::pointEigenToMsg(moved_along_z.translation(),p_end);
      z_axes.points.push_back(p_start);
      z_axes.points.push_back(p_end);

      // saving previous
      prev = pose;
    }

    line.points.push_back(p_start);
  }

  markers_msg.markers.push_back(x_axes);
  markers_msg.markers.push_back(y_axes);
  markers_msg.markers.push_back(z_axes);
  markers_msg.markers.push_back(line);

  marker_publisher_.publish(markers_msg);

}

void pathCallback(const tx90_moveit_client::scan_path msg)
{
  float x = msg.path_x[0];
  float y = msg.path_y[0];
  std::cout << "x: "<< x <<" y: " << y << std::endl;
}
// make cartesian point (path) with NO Tolerance on the TCP 
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt); 
// make cartesian point (path) with free rotation on the TCP
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt);

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_path");
	ros::NodeHandle nh;

	marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_trajectory_curve",1,true);
  traj_pub_         = nh.advertise<trajectory_msgs::JointTrajectory>("traj", 100);
  path_sub_         = nh.subscribe("/path", 100, pathCallback);
	// Required for communication with moveit components	
  ros::AsyncSpinner spinner (100);
  spinner.start();

	descartes_core::RobotModelPtr model (new descartes_moveit::IkFastMoveitStateAdapter());

  const std::string robot_description = "robot_description";
  const std::string group_name = "tx_90";
  const std::string world_frame = "base_link";
  const std::string tcp_frame = "tool0";

	if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
	{ROS_INFO("Could not initialize robot model");return -1;}
	// Let's turn on collision checking.
	model->setCheckCollisions(true); 
	//makePath
	std::vector<descartes_core::TrajectoryPtPtr> points = makePath();

	descartes_planner::DensePlanner planner;

	if (!planner.initialize(model))
	{ROS_ERROR("Failed to initialize planner");return -2;}	
  if (!planner.planPath(points))
	{ROS_ERROR("Could not solve for a valid path");return -3;}

	std::vector<descartes_core::TrajectoryPtPtr> result;

	if (!planner.getPath(result))
	{ROS_ERROR("Could not retrieve path");return -4;}

	std::vector<std::string> names;

	nh.getParam("controller_joint_names", names);

  names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

	trajectory_msgs::JointTrajectory joint_solution;
	joint_solution.joint_names = names;
	const static double default_joint_vel = 0.5; // rad/s
	
	if (!descartes_utilities::toRosJointPoints(*model, result, default_joint_vel, joint_solution.points))
	{ROS_ERROR("Unable to convert Descartes trajectory to joint points");return -5;}
	// 6. Send the ROS trajectory to the robot for execution
	if (!executeTrajectory(joint_solution))
	{ROS_ERROR("Could not execute trajectory!");return -6;}
	ROS_INFO("Done!");
	return 0;
}

std::vector<descartes_core::TrajectoryPtPtr> makePath()
{

  const static double step_size = 0.01;
  const static int num_steps = 20;
  const static double time_between_points = 0.5;

  // float path_x[98] = {0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.707, 0.707, 0.707, 0.707, 0.707, 0.707, 0.707, 0.707, 0.707, 0.707, 0.707, 0.707, 0.707, 0.707, 0.729, 0.729, 0.729, 0.729, 0.729, 0.729, 0.729, 0.729, 0.729, 0.729, 0.729, 0.729, 0.729, 0.729, 0.751, 0.751, 0.751, 0.751, 0.751, 0.751, 0.751, 0.751, 0.751, 0.751, 0.751, 0.751, 0.751, 0.751, 0.773, 0.773, 0.773, 0.773, 0.773, 0.773, 0.773, 0.773, 0.773, 0.773, 0.773, 0.773, 0.773, 0.773, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.817, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795, 0.795};
  // float path_y[98] = {-0.085, -0.074, -0.063, -0.052, -0.041, -0.03, -0.019, -0.008, 0.003, 0.014, 0.025, 0.036, 0.047, 0.047, 0.036, 0.025, 0.014, 0.003, -0.008, -0.019, -0.03, -0.041, -0.052, -0.063, -0.074, -0.085, -0.096, -0.096, -0.085, -0.074, -0.063, -0.052, -0.041, -0.03, -0.019, -0.008, 0.003, 0.014, 0.025, 0.036, 0.047, 0.047, 0.036, 0.025, 0.014, 0.003, -0.008, -0.019, -0.03, -0.041, -0.052, -0.063, -0.074, -0.085, -0.096, -0.096, -0.085, -0.074, -0.063, -0.052, -0.041, -0.03, -0.019, -0.008, 0.003, 0.014, 0.025, 0.036, 0.047, 0.047, 0.036, 0.025, 0.014, 0.003, -0.008, -0.019, -0.03, -0.041, -0.052, -0.063, -0.074, -0.085, -0.096, -0.096, -0.096, -0.085, -0.074, -0.063, -0.052, -0.041, -0.03, -0.019, -0.008, 0.003, 0.014, 0.025, 0.036, 0.047};
  float path_x[2] = {0.754, 0.756};
  float path_y[2] = {-0.026, -0.026};
  float z_value = 0.2; 
  int path_size = (sizeof(path_x)/sizeof(*path_x));
  std::cout << "path_size :" << path_size << std::endl;
  EigenSTL::vector_Isometry3d pattern_poses;
  for (int i = 0; i < path_size; i++)
  {
    std::cout << "i : " << i << std::endl;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(path_x[i]+0.068, path_y[i]+0.03, z_value);
    pose *=Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
    pattern_poses.push_back(pose);
  }

  // EigenSTL::vector_Isometry3d pattern_poses;
  // for (int i = -num_steps / 2; i < num_steps / 2; ++i)
  // {
  //   Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  //   pose.translation() = Eigen::Vector3d(i * step_size, 0, 0);
  //   pose *= Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());

  //   pattern_poses.push_back(pose);
  // }
  // for(int i=0; i<pattern_poses.size()*2; i++)
  // { std::cout << "I  :"   << i <<std::endl;
  //   std::cout << pattern_poses[i].translation() <<" front "<<std::endl;
  // }
  // Eigen::Isometry3d pattern_origin = Eigen::Isometry3d::Identity();
  // //set pattern start position
  // pattern_origin.translation() = Eigen::Vector3d(0.725, 0.086, 0.1); 

  std::vector<descartes_core::TrajectoryPtPtr> result;
  EigenSTL::vector_Isometry3d path_marker;
  for (const auto& pose : pattern_poses)
  { 
    Eigen::Isometry3d temp_path = pose; //pattern_origin * 
    path_marker.push_back(temp_path);
    descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pose, 0.1);//pattern_origin *
    result.push_back(pt);
  }
  publishPosesMarkers(path_marker);
  return result;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose), TimingConstraint(dt)) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI / 12.0, AxialSymmetricPt::Z_AXIS, TimingConstraint(dt)) );
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  traj_pub_.publish(trajectory);
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac ("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    std::cout << "problem here" << ac.waitForServer(ros::Duration(2.0)) << std::endl;
    ROS_ERROR("Could not connect to action server");
    return false;
  }
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(2.0); 
  
  return ac.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
}
