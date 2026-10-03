#!/usr/bin/env bash
# 08-b — Watchdog : publie /localization_ready si covariance < 0.05.
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
exec ros2 run caytu_nav_solution localization_watchdog
