search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=tx_90.srdf
robot_name_in_srdf=tx_90
moveit_config_pkg=tx90_moveit_config
robot_name=tx_90
planning_group_name=tx_90
ikfast_plugin_pkg=tx_90_ikfast_tx_90_plugin
base_link_name=base_link
eef_link_name=tool0
ikfast_output_path=/home/benlee/catkin_ws/src/tx_90_ikfast_tx_90_plugin/src/tx_90_tx_90_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
