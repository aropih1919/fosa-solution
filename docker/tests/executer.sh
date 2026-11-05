#!/usr/bin/env bash
# Compilation et tests dans un workspace jetable, sans accès au robot.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
mkdir -p "$HOME" /tmp/fosa-ws/src
cd /tmp/fosa-ws

paquets=(caytu_config caytu_course caytu_interfaces caytu_nav_bringup
         caytu_nav_solution caytu_perception)

conserver_rapports() {
    code=$?
    trap - EXIT
    if [[ -d log ]]; then cp -r log /rapports/colcon; fi
    printf '%s\n' "$code" > /rapports/code_sortie.txt
    exit "$code"
}
trap conserver_rapports EXIT

tar -C /depot --exclude='__pycache__' --exclude='.pytest_cache' \
    --exclude='*.egg-info' --exclude='build' --exclude='install' \
    --exclude='log' --exclude='models' -cf - "${paquets[@]}" \
    | tar -C src -xf -
# Copie explicitement réservée aux tests ; simulation/COLCON_IGNORE est conservé
# dans le dépôt et le faux parc_msgs n'est jamais installé sur l'hôte.
tar -C /depot/simulation --exclude='__pycache__' --exclude='build' \
    --exclude='install' --exclude='log' -cf - parc_msgs | tar -C src -xf -

python3 --version
dpkg-query -W 'ros-jazzy-*' > /rapports/versions-ros.txt
colcon build --symlink-install --executor sequential \
    --event-handlers console_direct+
source install/setup.bash

# Exécution explicite : inclut aussi les tests Python du paquet CMake et ceux
# qui sont ignorés sur un poste dépourvu de rclpy/messages générés.
tests=()
for paquet in "${paquets[@]}"; do
    if [[ -d "src/$paquet/test" ]]; then tests+=("src/$paquet/test"); fi
done
python3 -m pytest -ra --rootdir=/tmp/fosa-ws "${tests[@]}" \
    /opt/fosa-tests/test_installation_ros.py \
    --junitxml=/rapports/pytest.xml
