#!/usr/bin/env bash
# Prépare un workspace ROS 2 Jazzy de simulation dans Docker, sans toucher aux sources.
set -euo pipefail

depot="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
racine_ros="$(cd -- "$depot/../.." && pwd)"
sources_parc="$racine_ros/src"
workspace="${FOSA_SIM_WORKSPACE:-$depot/results/docker/workspace-simulation}"
image="fosa-simulation:jazzy"

paquets_parc=(parc_robot_bringup parc_robot_description)
for paquet in "${paquets_parc[@]}"; do
    if [[ ! -f "$sources_parc/$paquet/package.xml" ]]; then
        echo "Paquet PARC absent : $sources_parc/$paquet" >&2
        exit 1
    fi
done

if [[ -d "$workspace/src" ]] && find "$workspace/src" -mindepth 1 -print -quit | grep -q .; then
    echo "Le workspace existe déjà : $workspace/src" >&2
    echo "Pour le réutiliser, relancez avec FOSA_SIM_WORKSPACE vers un dossier vide." >&2
    exit 1
fi
mkdir -p "$workspace/src"
echo "Workspace : $workspace"

docker build --file "$depot/docker/simulation/Dockerfile" --tag "$image" "$depot"

docker run --rm --network bridge \
    --mount "type=bind,source=$depot,target=/depot,readonly" \
    --mount "type=bind,source=$sources_parc,target=/parc,readonly" \
    --mount "type=bind,source=$workspace,target=/workspace" \
    "$image" bash -lc '
        set -euo pipefail
        set +u
        source /opt/ros/jazzy/setup.bash
        set -u
        mkdir -p /workspace/src
        paquets=(caytu_config caytu_course caytu_interfaces caytu_nav_bringup
                 caytu_nav_solution caytu_perception)
        for paquet in "${paquets[@]}"; do
            cp -a "/depot/$paquet" /workspace/src/
        done
        cp -a /depot/simulation/parc_msgs /workspace/src/
        cp -a /parc/parc_robot_bringup /parc/parc_robot_description /workspace/src/

        if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
            rosdep init
        fi
        apt-get update -qq
        rosdep update --rosdistro jazzy
        rosdep install --from-paths /workspace/src --ignore-src \
            --rosdistro jazzy --skip-keys ament_python -r -y
        colcon build --symlink-install --executor sequential \
            --base-paths /workspace/src --event-handlers console_direct+
    '

echo "Workspace ROS préparé et compilé : $workspace"
echo "Pour l’utiliser dans un conteneur : source /opt/ros/jazzy/setup.bash && source $workspace/install/setup.bash"
