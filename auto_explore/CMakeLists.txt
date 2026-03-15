cmake_minimum_required(VERSION 3.8)
project(go2w_auto_explore)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Detect ROS distro for API compatibility (Foxy vs Humble+)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "foxy" OR $ENV{ROS_DISTRO} STREQUAL "galactic")
    set(ROS_FOXY_COMPAT TRUE)
  endif()
endif()

add_executable(frontier_explorer src/frontier_explorer.cpp)
target_compile_features(frontier_explorer PUBLIC cxx_std_17)
if(ROS_FOXY_COMPAT)
  target_compile_definitions(frontier_explorer PRIVATE ROS_FOXY_COMPAT=1)
endif()
ament_target_dependencies(
  frontier_explorer
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2_msgs
  visualization_msgs
)

install(DIRECTORY config launch rviz urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/slam_client.py
  scripts/scan_to_gridmap.py
  scripts/frontier_explorer.py
  scripts/frontier_explorer_foxy.py
  scripts/official_slam_odom_bridge.py
  scripts/wait_for_transform.py
  scripts/pointcloud_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  frontier_explorer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
