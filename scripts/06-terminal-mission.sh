#!/usr/bin/env bash
# 06-terminal-mission — Monitoring + watchdog + envoi du goal.
# Terminal 6 : ouvrir 3 onglets.
#   6a monitor  : ros2 run caytu_nav_solution nav_monitor
#   6b watchdog : ros2 run caytu_nav_solution localization_watchdog
#   6c goal     : ros2 run caytu_nav_solution task_solution --ros-args -p use_test_goal:=false
# Note Jour 3 : task_solution version Mpiaro attend AUSSI /camera_perception_ready
# (timeout 30 s -> mode degrade LiDAR seul). Ici version Johny = attente loc seule.
# Alternatif manuel : RViz 2D Goal Pose ou :
# ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
#   "{pose: {header: {frame_id: map}, pose: {position: {x: -2.29, y: 2.23}, orientation: {w: 1.0}}}}"
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
echo "=== 6a : monitoring ==="
echo 'ros2 run caytu_nav_solution nav_monitor'
echo "=== 6b : watchdog ==="
echo 'ros2 run caytu_nav_solution localization_watchdog'
echo "=== 6c : goal ==="
echo 'ros2 run caytu_nav_solution task_solution --ros-args -p use_test_goal:=false'
