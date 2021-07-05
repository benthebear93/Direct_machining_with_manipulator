cmake_minimum_required(VERSION 3.0.2)

project(tx90_moveit_client)

find_package(catkin REQUIRED COMPONENTS
  roslib
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  descartes_utilities
  trajectory_msgs

  moveit_commander
  rospy
  roscpp
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
)

catkin_package(INCLUDE_DIRS CATKIN_DEPENDS roscpp tf)

find_package(roslaunch)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tool_tf_listener src/tool_tf_listener.cpp)
target_link_libraries(tool_tf_listener ${catkin_LIBRARIES})
add_dependencies(tool_tf_listener tool_tf_listener)

add_executable(example1 src/example1.cpp)
target_link_libraries(example1 ${catkin_LIBRARIES})
add_dependencies(example1 example1)

add_executable(scan_trajgen src/scan_trajgen.cpp)
target_link_libraries(scan_trajgen ${catkin_LIBRARIES})
add_dependencies(scan_trajgen scan_trajgen)

add_executable(scan_trajgen_sub src/scan_trajgen_sub.cpp)
target_link_libraries(scan_trajgen_sub ${catkin_LIBRARIES})
add_dependencies(scan_trajgen_sub scan_trajgen_sub)