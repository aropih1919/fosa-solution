#!/usr/bin/env bash
# Simulation Gazebo de la course, sur le PC uniquement (jamais sur le robot).
#
#   simulation/simulation.sh preparer        une fois : parc_msgs de simulation + tags
#   simulation/simulation.sh essai [ordre]   course complète automatique (≈ 30-45 min)
#   simulation/simulation.sh monde           monde + robot + tags seuls (essais manuels)
#   simulation/simulation.sh moniteur        arbitre et juges simulés (essais manuels)
#
# Variables : FOSA_SIM_ODOM_CORRIGEE=1 (recommandé), FOSA_SIM_GUI=1 (affichage).
set -uo pipefail

DOSSIER_SIM="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPOT="$(cd -- "$DOSSIER_SIM/.." && pwd)"
ESPACE_TRAVAIL="$(cd -- "$DEPOT/../.." && pwd)"
CACHE="${FOSA_SIM_CACHE:-$HOME/.fosa_simulation}"
JOURNAUX="$CACHE/journaux"
export FOSA_SIM_TAGS="$CACHE/tags"
export FOSA_BASE_WS="$CACHE/parc_ws"      # parc_msgs de simulation, pour run.sh
export FOSA_PROFIL=simulation

sourcer() {
    set +u
    source /opt/ros/jazzy/setup.bash
    [[ -f "$ESPACE_TRAVAIL/install/setup.bash" ]] && source "$ESPACE_TRAVAIL/install/setup.bash"
    [[ -f "$FOSA_BASE_WS/install/setup.bash" ]] && source "$FOSA_BASE_WS/install/setup.bash"
    set -u
    export FOSA_CONFIG_DIR="$DEPOT/caytu_config/config"
    export FOSA_CONFIG_OVERRIDE="$FOSA_CONFIG_DIR/profils/simulation.yaml"
    if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
        ROS_DOMAIN_ID="$(python3 -c 'from caytu_config import load_config; print(load_config().system.ros_domain_id)')"
        export ROS_DOMAIN_ID
    fi
}

preparer() {
    mkdir -p "$FOSA_BASE_WS/src"
    ln -sfn "$DOSSIER_SIM/parc_msgs" "$FOSA_BASE_WS/src/parc_msgs"
    set +u; source /opt/ros/jazzy/setup.bash; set -u
    (cd "$FOSA_BASE_WS" && colcon build --event-handlers console_cohesion-) || exit 1
    python3 "$DOSSIER_SIM/generer_tags.py" "$FOSA_SIM_TAGS" || exit 1
    echo "Simulation prête dans $CACHE."
}

verifier_preparation() {
    if [[ ! -f "$FOSA_BASE_WS/install/setup.bash" || ! -d "$FOSA_SIM_TAGS/tag1" ]]; then
        echo "Lancez d'abord : simulation/simulation.sh preparer" >&2
        exit 1
    fi
}

arreter_groupe() {
    [[ -n "${1:-}" ]] && kill -INT -- "-$1" 2>/dev/null
}

essai() {
    local ordre="${1:-7,1,9,4}" duree_max="${2:-2700}"
    verifier_preparation
    sourcer
    mkdir -p "$JOURNAUX"
    local monde="$JOURNAUX/monde.log" course="$JOURNAUX/course.log" moniteur="$JOURNAUX/moniteur.log"
    echo "Essai : ordre $ordre, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, journaux dans $JOURNAUX"

    setsid ros2 launch "$DOSSIER_SIM/monde_tags.launch.py" > "$monde" 2>&1 &
    local pid_monde=$!
    sleep 40
    (cd "$DEPOT" && setsid ./run.sh course-premier-plan > "$course" 2>&1) &
    local pid_course=$!
    for _ in $(seq 120); do
        grep -q "ATTENTE_ORDRE" "$course" && break
        sleep 1
    done
    setsid python3 -u "$DOSSIER_SIM/moniteur.py" > "$moniteur" 2>&1 &
    local pid_moniteur=$!
    sleep 3
    (cd "$DEPOT" && timeout 20 ./run.sh ordre-test "$ordre" > /dev/null 2>&1)
    sleep 2
    (cd "$DEPOT" && timeout 20 ./run.sh depart > /dev/null 2>&1)
    echo "Départ donné. Suivi : tail -f $moniteur"

    local debut
    debut=$(date +%s)
    while (( $(date +%s) - debut < duree_max )); do
        grep -q '"etat": "TERMINEE"' "$moniteur" && break
        sleep 10
    done
    sleep 5
    arreter_groupe "$pid_moniteur"
    arreter_groupe "$pid_course"
    sleep 12
    arreter_groupe "$pid_monde"
    sleep 8
    pkill -f "[g]z sim -r" 2>/dev/null
    echo "Essai terminé. Résumé :"
    grep -aE "VISITE|ARBITRE|sauté|TERMINEE" "$moniteur" | cut -c1-200
    grep -aE "Plan :|sauté" "$course" | sed 's/\x1b\[[0-9;]*m//g' | cut -c40-220
}

case "${1:-}" in
    preparer) preparer ;;
    essai) shift; essai "$@" ;;
    monde) verifier_preparation; sourcer; exec ros2 launch "$DOSSIER_SIM/monde_tags.launch.py" ;;
    moniteur) verifier_preparation; sourcer; exec python3 -u "$DOSSIER_SIM/moniteur.py" ;;
    *) sed -n '2,9p' "$0"; exit 2 ;;
esac
