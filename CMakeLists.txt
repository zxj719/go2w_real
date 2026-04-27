cmake_minimum_required(VERSION 3.8)
project(go2w_real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(
  EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/frontier_goal_selector.cpp" AND
  EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/frontier_explorer.cpp" AND
  EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/frontier_search.cpp"
)
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
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

set(package_share_dirs "")
foreach(dir IN ITEMS config launch urdf meshes rviz behavior_trees)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    list(APPEND package_share_dirs "${dir}")
  endif()
endforeach()

if(package_share_dirs)
  install(DIRECTORY ${package_share_dirs}
    DESTINATION share/${PROJECT_NAME}
  )
endif()

set(package_scripts "")
foreach(script IN ITEMS
  scripts/go2w_bridge.py
  scripts/go2w_motion_executor.py
  scripts/navigation_debug_bag_topics.sh
  scripts/navigate_to_waypoint.py
  scripts/navigate_to_waypoint_with_debug_bag.sh
  scripts/navigation_headless_config.py
  scripts/navigation_executor.py
  scripts/navigation_sender_test_poi001.py
  scripts/profile_nav_stack.py
  scripts/pointcloud_relay.py
  scripts/save_map_and_waypoints.py
  scripts/start_navigation_headless.sh
  scripts/wait_for_transform.py
)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${script}")
    list(APPEND package_scripts "${script}")
  endif()
endforeach()

if(package_scripts)
  install(PROGRAMS ${package_scripts}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_python_install_package(go2w_real)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_motion_executor_types
    test/test_motion_executor_types.py
  )
  ament_add_pytest_test(
    test_motion_executor_geometry
    test/test_motion_executor_geometry.py
  )
  ament_add_pytest_test(
    test_motion_executor_logic
    test/test_motion_executor_logic.py
  )
  ament_add_pytest_test(
    test_motion_executor_launch_config
    test/test_motion_executor_launch_config.py
  )
  ament_add_pytest_test(
    test_go2w_bridge_deadband
    test/test_go2w_bridge_deadband.py
  )
  ament_add_pytest_test(
    test_nav2_controller_config
    test/test_nav2_controller_config.py
  )
  ament_add_pytest_test(
    test_odom_fusion_config
    test/test_odom_fusion_config.py
  )
  ament_add_pytest_test(
    test_navigation_executor
    test/test_navigation_executor.py
  )
  ament_add_pytest_test(
    test_headless_config
    test/test_headless_config.py
  )
  ament_add_pytest_test(
    test_start_navigation_headless
    test/test_start_navigation_headless.py
  )
  ament_add_pytest_test(
    test_navigate_to_waypoint
    test/test_navigate_to_waypoint.py
  )
  ament_add_pytest_test(
    test_navigate_to_waypoint_debug_bag
    test/test_navigate_to_waypoint_debug_bag.py
  )
  ament_add_pytest_test(
    test_wait_for_transform
    test/test_wait_for_transform.py
  )
  ament_add_pytest_test(
    test_save_map_and_waypoints
    test/test_save_map_and_waypoints.py
  )
endif()

ament_package()
