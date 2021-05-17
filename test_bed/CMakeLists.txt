cmake_minimum_required(VERSION 2.8.3)

project(test_bed)

find_package(catkin REQUIRED COMPONENTS 
	roslaunch 
	message_generation 
	roscpp pcl_conversions 
	pcl_ros std_msgs 
	sensor_msgs
)

catkin_package(INCLUDE_DIRS CATKIN_DEPENDS 
	roscpp 
	pcl_conversions 
	pcl_ros
)

find_package(roslaunch)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(preprocess_pointcloud src/preprocess_pointcloud.cpp)
target_link_libraries(preprocess_pointcloud ${catkin_LIBRARIES})
add_dependencies(preprocess_pointcloud preprocess_pointcloud)

add_executable(find_normal src/find_normal.cpp)
target_link_libraries(find_normal ${catkin_LIBRARIES})
add_dependencies(find_normal find_normal)

add_executable(normal_visualize src/normal_visualize.cpp)
target_link_libraries(normal_visualize ${catkin_LIBRARIES})
add_dependencies(normal_visualize normal_visualize)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
