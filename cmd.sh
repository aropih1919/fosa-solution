#!/usr/bin/env bash

# Lancez avec : ./cmd.sh gazebo | benchmark | compare
# gazebo    : terminal 1, monde PARC/Gazebo (sorties masquées).
# benchmark : terminal 2, menu du laboratoire sur le monde déjà lancé.
# compare   : affiche les rapports stockés dans results/.

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "$SCRIPT_DIR/../.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy introuvable : /opt/ros/jazzy/setup.bash" >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash

if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    echo "Workspace non compilé : $WORKSPACE_DIR/install/setup.bash absent." >&2
    echo "Compilez d'abord les packages Fosa avec colcon, puis relancez ce script." >&2
    exit 1
fi
source "$WORKSPACE_DIR/install/setup.bash"
set -u

ensure_fosa_executable() {
    local executable="$1"
    local needs_build=false
    local package_prefix=""
    local installed_executable=""
    local newer_source=""

    if ! ros2 pkg executables caytu_nav_solution 2>/dev/null \
        | awk -v wanted="$executable" \
            '$1 == "caytu_nav_solution" && $2 == wanted { found = 1 } END { exit !found }'; then
        needs_build=true
    else
        package_prefix="$(ros2 pkg prefix caytu_nav_solution)"
        installed_executable="$package_prefix/lib/caytu_nav_solution/$executable"
        if [[ ! -x "$installed_executable" ]]; then
            needs_build=true
        else
            newer_source="$(find "$SCRIPT_DIR/caytu_nav_solution" \
                "$SCRIPT_DIR/caytu_nav_bringup" -type f \
                \( -name '*.py' -o -name '*.yaml' -o -name '*.xml' \
                   -o -name 'setup.py' -o -name 'CMakeLists.txt' \) \
                -newer "$installed_executable" -print -quit)"
            [[ -z "$newer_source" ]] || needs_build=true
        fi
    fi

    if [[ "$needs_build" != true ]]; then
        return 0
    fi
    echo "L'exécutable ROS 2 '$executable' est absent ou obsolète; compilation des packages Fosa..."
    (
        cd "$WORKSPACE_DIR"
        colcon build --symlink-install \
            --packages-select caytu_nav_bringup caytu_nav_solution
    )
    set +u
    source "$WORKSPACE_DIR/install/setup.bash"
    set -u
}

mode="${1:-}"
if [[ $# -gt 0 ]]; then
    shift
fi

case "$mode" in
    gazebo)
        echo "Terminal 1 : lancement de Gazebo. Les sorties sont redirigées vers /dev/null."
        echo "Laissez ce terminal ouvert; Ctrl+C arrête le monde."
        ros2 launch parc_robot_bringup task.launch.py >/dev/null 2>&1
        ;;
    benchmark)
        ensure_fosa_executable fosa_benchmark
        echo "Terminal 2 : lancement du benchmark sur le monde déjà ouvert."
        echo "Sans options, le sélecteur interactif vous guidera."
        ros2 run caytu_nav_solution fosa_benchmark --ros-args \
            -p start_simulation:=false "$@"
        ;;
    compare)
        ensure_fosa_executable fosa_compare
        ros2 run caytu_nav_solution fosa_compare \
            --results-dir "$SCRIPT_DIR/results" "$@"
        ;;
    *)
        cat <<'USAGE'
Usage : ./cmd.sh MODE [options]

À exécuter dans deux terminaux :
  Terminal 1 : ./cmd.sh gazebo
  Terminal 2 : ./cmd.sh benchmark

Exemple non interactif dans le terminal 2 :
  ./cmd.sh benchmark -p controller:=rpp -p localization:=odom_imu \
    -p profile:=balanced -p scenario:=full_course

Autres commandes :
  ./cmd.sh compare [options fosa_compare]
USAGE
        exit 2
        ;;
esac
