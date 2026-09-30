cmake_minimum_required(VERSION 3.0.2)
project(c12_ros1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  cv_bridge
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/c12_gimbal_node.py
  scripts/c12_image_bridge_node.py
  scripts/c12_tag_pose_node.py
  scripts/c12_tag_localization_node.py
  scripts/c12_tag_mission_node.py
  scripts/odom_tf_bridge_node.py
  scripts/static_tf_from_params_node.py
  scripts/tag_world_stability_check_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
