cmake_minimum_required(VERSION 3.8)
project(go2w_real)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/go2w_bridge.py
  scripts/go2_motion_command.py
  scripts/multi_source_odom_bridge.py
  scripts/official_slam_odom_bridge.py
  scripts/pointcloud_relay.py
  scripts/save_map_and_waypoints.py
  scripts/wait_for_transform.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
