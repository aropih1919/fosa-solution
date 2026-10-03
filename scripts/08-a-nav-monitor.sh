#!/usr/bin/env bash
# 08-a — Monitoring : logs recherche chemin, trouve, avance, rotation.
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
exec ros2 run caytu_nav_solution nav_monitor
