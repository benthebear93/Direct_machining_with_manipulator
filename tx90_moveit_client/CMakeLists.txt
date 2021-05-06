cmake_minimum_required(VERSION 3.0.2)

project(tx90_moveit_client)

find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  rospy
  roscpp
)

catkin_package(INCLUDE_DIRS CATKIN_DEPENDS roscpp pcl_conversions pcl_ros)

find_package(roslaunch)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME}_node src/tx90_moveit_client_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

