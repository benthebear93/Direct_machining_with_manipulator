cmake_minimum_required(VERSION 3.0.2)

project(tx90_moveit_client)

find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  rospy
  roscpp
  geometry_msgs
  tf
)

catkin_package(INCLUDE_DIRS CATKIN_DEPENDS roscpp tf)

find_package(roslaunch)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tool_tf_listener src/tool_tf_listener.cpp)
target_link_libraries(tool_tf_listener ${catkin_LIBRARIES})
add_dependencies(tool_tf_listener tool_tf_listener)