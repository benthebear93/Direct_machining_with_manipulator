# Direct machining on 3D constructed freeform surface using point cloud with manipulator(staubli tx90)

This repository is for simulation of direct machining on unknown freeform surface.

Line scanner and depth camera that attached to manipulator used to reconstruct 3d surface to estimate better normal vector for drilling point. To reconstruct 3d surface, depth camera is being used to roughly estimate the ROI of product that need to be drilled. With this rough data, optimal scanning path for manipulator with line scanner is calculated. 

## [Simulation]

roslaunch staubli_tx90_gazebo tx90_gazebo.launch                            (launch gazebo env)

roslaunch tx90_moveit_config move_group.launch                              (launch move_group)

rosrun tx90_moveit_client client_tutorial.py                                          (moveit client script)

rosrun rviz rviz -d `rospack find test_bed`/rviz/tf_marker.rviz           (rviz setting)

rosrun test_bed pointcloud_drillpoint.py                                               (select drilling point)

rosrun tx90_moveit_client tool_tf_listener                                            (pub marker for rviz & print tf)

rosrun test_bed pcd_to_rviz                                                                 (pcd file stream to rviz)

rosrun test_bed preprocess_pointcloud                                               (preprocess for pcd)

rosrun tf static_traform_publisher 0.619517 0.0324866 1.49232 -1.5707 3.141592654 0 world seg_pcd 100                                                                                         (static tf for pcd from world)

## [Real robot]

txreal

roslaunch staubli_tx90_gazebo spawn_tx90.launch

rviz

roslaunch tx90_moveit_config move_group.launch

tx90_c

rosrun test_bed pcd_to_rviz

rosrun tf static_transform_publisher 0.619517 0.0324866 1.49232 -1.5707 3.141592654 0 world pcd 100
