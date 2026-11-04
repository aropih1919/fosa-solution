#!/usr/bin/env bash
# Simulation Gazebo de la course, sur le PC uniquement (jamais sur le robot).
#
#   simulation/simulation.sh preparer          une fois : parc_msgs de simulation + tags
#   simulation/simulation.sh voix-preparer     une fois : Vosk + modèle FR (départ vocal)
#   simulation/simulation.sh essai [ordre]     répétition du jour J : préparation puis match
#   simulation/simulation.sh preparation       préparation seule (carte + tags enregistrés)
#   simulation/simulation.sh course [ordre]    match seul (avec la préparation si elle existe)
#   simulation/simulation.sh monde             monde + robot + tags seuls (essais manuels)
#   simulation/simulation.sh moniteur          récit + arbitre et juges simulés
#   simulation/simulation.sh micro             micro du PC -> /speech_audio (essais manuels)
#   simulation/simulation.sh arreter           tout arrêter (Gazebo, course, moniteur)
#
# Gazebo s'affiche (FOSA_SIM_GUI=0 pour le masquer, plus rapide).
# FOSA_SIM_ODOM_CORRIGEE=1 : odométrie du modèle corrigée (recommandé).
# FOSA_SIM_VOIX=1 : au match, départ à la voix dans le micro du PC (« allez »).
set -uo pipefail

DOSSIER_SIM="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPOT="$(cd -- "$DOSSIER_SIM/.." && pwd)"
ESPACE_TRAVAIL="$(cd -- "$DEPOT/../.." && pwd)"
CACHE="${FOSA_SIM_CACHE:-$HOME/.fosa_simulation}"
JOURNAUX="$CACHE/journaux"
export FOSA_SIM_TAGS="$CACHE/tags"
export FOSA_BASE_WS="$CACHE/parc_ws"      # parc_msgs de simulation, pour run.sh
export FOSA_PROFIL=simulation
export FOSA_SIM_GUI="${FOSA_SIM_GUI:-1}"
DUREE_MAX="${FOSA_SIM_DUREE_MAX:-7200}"   # secondes réelles par phase (Gazebo peut être 10 fois plus lent)

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

MODELE_VOSK="vosk-model-small-fr-0.22"

voix_preparer() {
    local cible="$ESPACE_TRAVAIL/.fosa_python" modeles="$DEPOT/caytu_config/config/models"
    echo "[1/3] Vosk dans $cible (sans sudo)..."
    PYTHONPATH="$cible" python3 -c 'import vosk' 2>/dev/null \
        || python3 -m pip install --quiet --target "$cible" vosk || exit 1
    echo "[2/3] Modèle français $MODELE_VOSK..."
    if [[ ! -d "$modeles/$MODELE_VOSK" ]]; then
        mkdir -p "$modeles" "$CACHE"
        curl -fsSL "https://alphacephei.com/vosk/models/$MODELE_VOSK.zip" \
            -o "$CACHE/$MODELE_VOSK.zip" || exit 1
        python3 -m zipfile -e "$CACHE/$MODELE_VOSK.zip" "$modeles" || exit 1
    fi
    echo "[3/3] Messages audio de ROS..."
    set +u; source /opt/ros/jazzy/setup.bash; set -u
    if python3 -c 'import audio_common_msgs.msg' 2>/dev/null; then
        echo "Départ vocal prêt. Micros disponibles : arecord -L (choix : FOSA_SIM_MICRO)."
    else
        echo "Il manque un paquet système :  sudo apt install ros-jazzy-audio-common-msgs" >&2
        exit 1
    fi
}

verifier_preparation() {
    if [[ ! -f "$FOSA_BASE_WS/install/setup.bash" || ! -f "$FOSA_SIM_TAGS/tag1/tag1.obj" ]]; then
        echo "Lancez d'abord : simulation/simulation.sh preparer" >&2
        exit 1
    fi
}

# Une course ou un Gazebo restés d'un essai précédent pilotent le même robot
# simulé et faussent tout : on refuse de démarrer.
verifier_rien_ne_tourne() {
    if pgrep -f "[c]ourse_parc|[g]z sim|[c]ourse.launch.py|[t]ag_detector|[d]epth_scan" >/dev/null; then
        echo "Une course ou Gazebo tourne déjà : simulation/simulation.sh arreter" >&2
        exit 1
    fi
}

arreter_groupe() {
    if [[ -n "${1:-}" ]]; then kill -INT -- "-$1" 2>/dev/null; fi
}

arreter() {
    pkill -INT -f "[c]ourse.launch.py"
    pkill -INT -f "[m]onde_tags.launch.py"
    pkill -INT -f "[m]oniteur.py"
    pkill -INT -f "[m]icro_pc.py"
    sleep 10
    # Lanceurs d'abord : ils relanceraient les nœuds de perception (respawn).
    pkill -9 -f "[c]ourse.launch.py"
    pkill -9 -f "[m]onde_tags.launch.py"
    local noeud
    for noeud in course_parc mot_depart tag_detector depth_scan scan_to_scan_filter_chain \
        slam_toolbox controller_server planner_server behavior_server bt_navigator \
        lifecycle_manager robot_state_publisher parameter_bridge image_bridge "gz sim"; do
        pkill -9 -f "[${noeud:0:1}]${noeud:1}"
    done
    echo "Simulation arrêtée."
}

# Une phase complète dans un monde neuf (robot à la position de départ) :
# Gazebo, course (match ou préparation), puis le moniteur au premier plan
# jusqu'au résumé final.
phase() {
    local mode="$1" ordre="${2:-}"
    verifier_rien_ne_tourne
    mkdir -p "$JOURNAUX"
    local monde="$JOURNAUX/monde_$mode.log" course="$JOURNAUX/course_$mode.log"
    local moniteur="$JOURNAUX/moniteur_$mode.log"
    echo "=== ${mode^^} (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, journaux : $JOURNAUX) ==="
    setsid ros2 launch "$DOSSIER_SIM/monde_tags.launch.py" > "$monde" 2>&1 &
    local pid_monde=$!
    echo "Démarrage de Gazebo (environ 40 s)..."
    sleep 40
    local arguments=()
    [[ "$mode" == preparation ]] && arguments=(mode:=preparation)
    # setsid lancé directement : $! est le chef du groupe à arrêter ensuite.
    setsid "$DEPOT/run.sh" course-premier-plan "${arguments[@]}" > "$course" 2>&1 &
    local pid_course=$!
    local pid_micro=""
    if [[ "$mode" == course && "${FOSA_SIM_VOIX:-0}" == 1 ]]; then
        setsid python3 -u "$DOSSIER_SIM/micro_pc.py" > "$JOURNAUX/micro.log" 2>&1 &
        pid_micro=$!
        echo ">>> Départ À LA VOIX : quand le récit affiche « À l'écoute du mot de départ »,"
        echo ">>> dites « allez » distinctement dans le micro du PC."
    fi
    if [[ "$mode" == course ]]; then
        (
            for _ in $(seq 120); do
                grep -q "ATTENTE_ORDRE" "$course" && break
                sleep 1
            done
            sleep 3
            (cd "$DEPOT" && timeout 20 ./run.sh ordre-test "$ordre" > /dev/null 2>&1)
            sleep 2
            if [[ "${FOSA_SIM_VOIX:-0}" != 1 ]]; then
                (cd "$DEPOT" && timeout 20 ./run.sh depart > /dev/null 2>&1)
            fi
        ) &
    fi
    timeout "$DUREE_MAX" python3 -u "$DOSSIER_SIM/moniteur.py" --quitter-a-la-fin \
        | tee "$moniteur"
    echo "Arrêt de la phase ${mode}..."
    arreter_groupe "$pid_micro"
    arreter_groupe "$pid_course"
    sleep 5
    arreter_groupe "$pid_monde"
    # Le lanceur ROS ne finit pas toujours son arrêt (nœuds relancés par
    # respawn) : arrêt complet, sinon la phase suivante refuserait de démarrer.
    arreter > /dev/null
}

case "${1:-}" in
    preparer) preparer ;;
    voix-preparer) voix_preparer ;;
    arreter) arreter ;;
    micro) verifier_preparation; sourcer; exec python3 -u "$DOSSIER_SIM/micro_pc.py" ;;
    essai)
        verifier_preparation; sourcer
        phase preparation && phase course "${2:-7,1,9,4}"
        ;;
    preparation) verifier_preparation; sourcer; phase preparation ;;
    course) verifier_preparation; sourcer; phase course "${2:-7,1,9,4}" ;;
    monde) verifier_preparation; sourcer; exec ros2 launch "$DOSSIER_SIM/monde_tags.launch.py" ;;
    moniteur) verifier_preparation; sourcer; exec python3 -u "$DOSSIER_SIM/moniteur.py" ;;
    *) sed -n '2,16p' "$0"; exit 2 ;;
esac
