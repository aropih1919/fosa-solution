#!/usr/bin/env bash
# mission.sh — Menu unique dans l'ordre d'execution : choisir un numero, rien a memoriser.
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

MENU="00-build 01-gazebo 02-filtre 03-map 03b-map-activate 04-amcl 04b-amcl-activate 05-nav2 06-perception 07-surcouche 08a-monitor 08b-watchdog 08c-goal ordre-complet quitter"
PS3="Numero> "

select choix in $MENU; do
  case "$choix" in
    00-build)            exec "$SCRIPT_DIR/00-build.sh";;
    01-gazebo)           exec "$SCRIPT_DIR/01-terminal-gazebo.sh";;
    02-filtre)           exec "$SCRIPT_DIR/02-terminal-lidar-filter.sh";;
    03-map)              exec "$SCRIPT_DIR/03-terminal-map-server.sh";;
    03b-map-activate)    "$SCRIPT_DIR/03-b-map-activate.sh";;
    04-amcl)             exec "$SCRIPT_DIR/04-terminal-amcl.sh";;
    04b-amcl-activate)   "$SCRIPT_DIR/04-b-amcl-activate.sh";;
    05-nav2)             exec "$SCRIPT_DIR/05-terminal-nav2.sh";;
    06-perception)       exec "$SCRIPT_DIR/06-terminal-perception.sh";;
    07-surcouche)        exec "$SCRIPT_DIR/07-terminal-surcouche.sh";;
    08a-monitor)         exec "$SCRIPT_DIR/08-a-nav-monitor.sh";;
    08b-watchdog)        exec "$SCRIPT_DIR/08-b-localization-watchdog.sh";;
    08c-goal)            exec "$SCRIPT_DIR/08-c-task-goal.sh";;
    ordre-complet)
      echo "00 build -> 01 gazebo -> 02 filtre -> 03 map -> 03b activate -> 04 amcl -> 04b activate (+pose RViz)"
      echo "      -> 05 nav2 -> 06 perception -> 07 surcouche -> 08a monitor + 08b watchdog -> 08c goal";;
    quitter|"")          exit 0;;
  esac
done
