#!/usr/bin/env bash

GO2W_NAVIGATION_DEBUG_BAG_TOPICS=(
  /tf
  /tf_static
  /map
  /odom
  /rf2o/odom
  /sport_odom
  /sport_imu
  /lowstate_imu
  /utlidar_imu_base
  /unitree/slam_lidar/points
  /cloud_relayed
  /scan
  /scan_raw
  /cmd_vel
  /nav2_cmd_vel_raw
  /go2w_motion_executor/shadow_cmd_vel
  /go2w_motion_executor/navigation_status
  /navigate_to_pose/_action/goal
  /navigate_to_pose/_action/result
  /navigate_to_pose/_action/cancel
  /navigate_to_pose/_action/status
  /navigate_to_pose/_action/feedback
)
