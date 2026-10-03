#!/usr/bin/env bash
# 00-build — Build des seuls packages utiles (pas de build complet : le package
# organisateurs parc_robot exige ament_lint_auto non installe et fait echouer
# tout le workspace alors que personne ne le lance directement).
set -e
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  parc_robot_description parc_robot_bringup \
  caytu_nav_bringup caytu_nav_solution
source install/setup.bash
echo "[OK] Build termine. Sourcez install/setup.bash dans chaque nouveau terminal."
echo "(Optionnel, pour un build complet : sudo apt install -y ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common)"
