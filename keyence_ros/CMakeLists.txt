cmake_minimum_required(VERSION 2.8.3)
project(keyence_ros)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs tf tf2_ros pcl_conversions pcl_ros genmsg sensor_msgs message_generation)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

## Declare ROS messages and services
add_message_files(FILES keyence_laser.msg)
add_message_files(FILES point_pptk.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(INCLUDE_DIRS CATKIN_DEPENDS roscpp pcl_conversions pcl_ros DEPENDS Boost)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(keyence_profile src/keyence_profile.cpp)
target_link_libraries(keyence_profile ${catkin_LIBRARIES})
add_dependencies(keyence_profile keyence_profile_generate_messages_cpp)

add_executable(pcl_to_csv src/pcl_to_csv.cpp)
target_link_libraries(pcl_to_csv ${catkin_LIBRARIES})
add_dependencies(pcl_to_csv pcl_to_csv_generate_messages_cpp)

add_executable(gen_surface src/gen_surface.cpp)
target_link_libraries(gen_surface ${catkin_LIBRARIES})
add_dependencies(gen_surface gen_surface_generate_messages_cpp)

