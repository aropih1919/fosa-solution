#!/usr/bin/env bash
# mission.sh — Menu unique : plus rien a memoriser, choisir un numero.
# Chaque entree rappelle l'ordre et lance le bon script (ou affiche les commandes).
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

MENU="build-cible gazebo lidar-filter map-server amcl nav2 perception-camera surcouche aide-mission ordre-complet quitter"
PS3="Numero> "

select choix in $MENU; do
  case "$choix" in
    build-cible)      exec "$SCRIPT_DIR/00-build.sh";;
    gazebo)           exec "$SCRIPT_DIR/01-terminal-gazebo.sh";;
    lidar-filter)     exec "$SCRIPT_DIR/02-terminal-lidar-filter.sh";;
    map-server)
      echo "Apres le demarrage, 2e onglet : ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate"
      sleep 2; exec "$SCRIPT_DIR/03-terminal-map-server.sh";;
    amcl)
      echo "Puis : ros2 lifecycle set /amcl configure && ros2 lifecycle set /amcl activate"
      echo "Puis RViz : Fixed Frame=map, 2D Pose Estimate en (-0.2,-7.4) vers le nord."
      sleep 2; exec "$SCRIPT_DIR/04-terminal-amcl.sh";;
    nav2)             exec "$SCRIPT_DIR/05-terminal-nav2.sh";;
    perception-camera)
      echo "Lance filtre hauteur + watchdog camera (branche locale)."
      sleep 2
      ros2 launch caytu_camera_perception perception.launch.py;;
    surcouche)
      echo "APRES le 05 : sans ce terminal le robot ne recoit aucune consigne."
      sleep 2; exec "$SCRIPT_DIR/07-terminal-surcouche.sh";;
    aide-mission)
      echo "6a monitor  : ros2 run caytu_nav_solution nav_monitor"
      echo "6b watchdog : ros2 run caytu_nav_solution localization_watchdog"
      echo "6c goal     : ros2 run caytu_nav_solution task_solution --ros-args -p use_test_goal:=false";;
    ordre-complet)
      echo "Ordre : build -> 01 gazebo -> 02 filtre -> 03 map -> 04 amcl (+pose initiale)"
      echo "     -> 05 nav2 -> perception (camera) -> 07 surcouche -> 6a/6b/6c mission";;
    quitter|"")       exit 0;;
  esac
done
