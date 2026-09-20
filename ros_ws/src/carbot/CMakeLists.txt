cmake_minimum_required(VERSION 2.8.3)
project(carbot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/TunnelNav.cfg
  cfg/LaneFollow.cfg
  cfg/ParkingNav.cfg
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/lidar_controller.py
  scripts/camera_node.py
  scripts/yolo_detector.py
  scripts/shutdown_listener.py
  scripts/lidar_filter.py
  scripts/parking_nav.py
  scripts/tunnel_nav.py
  scripts/tunnel_nav_dashboard.py
  scripts/lane_follow.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
