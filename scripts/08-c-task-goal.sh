#!/usr/bin/env bash
# 08-c — Envoi du goal (lit task_params.yaml : x=-2.293 y=2.232). Dernier a lancer.
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
exec ros2 run caytu_nav_solution task_solution --ros-args -p use_test_goal:=false
