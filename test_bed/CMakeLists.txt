cmake_minimum_required(VERSION 2.8.3)

project(test_bed)

find_package(catkin REQUIRED COMPONENTS 
	roslaunch 
	message_generation 
	roscpp pcl_conversions 
	pcl_ros std_msgs 
	sensor_msgs
	roslib
	descartes_core
	descartes_moveit
	descartes_trajectory
	descartes_planner
	geometry_msgs
	trajectory_msgs
	visualization_msgs
	eigen_conversions
	tf
)
add_message_files(FILES boundary.msg )
add_message_files(FILES scan_path.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(INCLUDE_DIRS include 
	LIBRARIES descartes_core descartes_moveit descartes_planner
	CATKIN_DEPENDS descartes_core descartes_moveit descartes_planner geometry_msgs roscpp tf trajectory_msgs visualization_msgs descartes_trajectory
	roscpp 
	pcl_conversions 
	pcl_ros
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(roslaunch)

catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
  	${Eigen_INCLUDE_DIRS} 
  	include ${catkin_INCLUDE_DIRS}
  	)

#add_executable(path_traj_gen src/path_traj_gen.cpp)
#target_link_libraries(path_traj_gen ${catkin_LIBRARIES})
#add_dependencies(path_traj_gen path_traj_gen)

add_executable(preprocess_pointcloud src/preprocess_pointcloud.cpp)
target_link_libraries(preprocess_pointcloud ${catkin_LIBRARIES})
add_dependencies(preprocess_pointcloud preprocess_pointcloud)

add_executable(find_normal src/find_normal.cpp)
target_link_libraries(find_normal ${catkin_LIBRARIES})
add_dependencies(find_normal find_normal)

add_executable(normal_visualize src/normal_visualize.cpp)
target_link_libraries(normal_visualize ${catkin_LIBRARIES})
add_dependencies(normal_visualize normal_visualize)

add_executable(pcd_to_rviz src/pcd_to_rviz.cpp)
target_link_libraries(pcd_to_rviz ${catkin_LIBRARIES})
add_dependencies(pcd_to_rviz pcd_to_rviz)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
