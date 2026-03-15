cmake_minimum_required(VERSION 3.8)
project(go2w_real)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/go2w_bridge.py
  scripts/official_slam_odom_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
