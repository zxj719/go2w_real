cmake_minimum_required(VERSION 3.8)
project(go2w_real)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf meshes rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/go2w_bridge.py
  scripts/navigate_to_waypoint.py
  scripts/navigation_executor.py
  scripts/navigation_sender_test_poi001.py
  scripts/profile_nav_stack.py
  scripts/pointcloud_relay.py
  scripts/save_map_and_waypoints.py
  scripts/wait_for_transform.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
