cmake_minimum_required(VERSION 3.8)
project(go2w_real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- Frontier Goal Selector library ---
add_library(frontier_goal_selector_lib
  src/frontier_goal_selector.cpp
)
target_include_directories(frontier_goal_selector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(frontier_goal_selector_lib PUBLIC cxx_std_17)

# --- Frontier Explorer C++ node ---
add_executable(frontier_explorer_cpp
  src/frontier_explorer.cpp
  src/frontier_search.cpp
)
target_include_directories(frontier_explorer_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(frontier_explorer_cpp PUBLIC cxx_std_17)
target_link_libraries(frontier_explorer_cpp frontier_goal_selector_lib)
ament_target_dependencies(frontier_explorer_cpp
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  std_msgs
  builtin_interfaces
  tf2_msgs
  visualization_msgs
)

install(TARGETS frontier_explorer_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

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
  scripts/start_navigation_headless.sh
  scripts/wait_for_transform.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
