#!/usr/bin/env bash
# 05-terminal-nav2 — Nav2 (planner + controller + BT), tuning vitesse Jour 2 Johny.
# Terminal 5 : controller 0.55 m/s, planner Smac rapide, costmaps allegees.
# Benchmark Faneva conseille avant/apres : comparer temps, distance, recoveries.
# Diag : ros2 topic hz /scan_filtered ; ros2 topic hz /plan ; ros2 lifecycle get /controller_server
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch caytu_nav_bringup navigation.launch.py use_sim_time:=true
