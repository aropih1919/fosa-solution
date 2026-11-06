#!/usr/bin/env bash
# Simulation Gazebo de la course, sur le PC uniquement (jamais sur le robot).
#
#   simulation/simulation.sh preparer              construit les paquets officiels
#   simulation/simulation.sh voix-preparer        Vosk + modèle FR (départ vocal)
#   simulation/simulation.sh essai [ordre] [graine]
#                                                 préparation puis match, même arène
#   simulation/simulation.sh preparation [graine] préparation seule (carte + tags)
#   simulation/simulation.sh course [ordre] [graine]
#                                                 match seul (avec la préparation)
#   simulation/simulation.sh monde [graine]       monde + robot seuls (essais manuels)
#   simulation/simulation.sh moniteur [graine]    récit + arbitre et juges simulés
#   simulation/simulation.sh micro                micro du PC -> topics ReSpeaker
#   simulation/simulation.sh arreter              tout arrêter
#
# L'arène vient du générateur officiel : même graine = même disposition.
# Sans ordre, l'arbitre tire l'ordre au hasard (comme en compétition).
# FOSA_SIM_GUI=0 masque Gazebo ; FOSA_SIM_VOIX=1 : départ à la voix (« allez »).
# FOSA_SIM_AUDIO=1 publie le micro du PC (automatique en mode voix).
# FOSA_SIM_DUREE_MAX : secondes réelles par phase (défaut 7200).
# FOSA_SIM_GZ_PARTITION isole Gazebo des autres simulations sur le PC.
set -uo pipefail

DOSSIER_SIM="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPOT="$(cd -- "$DOSSIER_SIM/.." && pwd)"
ESPACE_TRAVAIL="$(cd -- "$DEPOT/../.." && pwd)"
CACHE="${FOSA_SIM_CACHE:-$HOME/.fosa_simulation}"
JOURNAUX="$CACHE/journaux"
MONDES="$CACHE/mondes"
export FOSA_SIM_GUI="${FOSA_SIM_GUI:-1}"
export FOSA_SIM_AUDIO="${FOSA_SIM_AUDIO:-0}"
export FOSA_SIM_MICRO="${FOSA_SIM_MICRO:-default}"
export GZ_PARTITION="${FOSA_SIM_GZ_PARTITION:-fosa_simulation_${UID:-$(id -u)}_$$}"
DUREE_MAX="${FOSA_SIM_DUREE_MAX:-7200}"
GRAINE_DEFAUT=7
JETON_ARBITRE="${FOSA_SIM_JETON:-parc2026}"
MODELE_VOSK="vosk-model-small-fr-0.22"

sourcer() {
    set +u
    source /opt/ros/jazzy/setup.bash
    if [[ ! -f "$ESPACE_TRAVAIL/install/setup.bash" ]]; then
        echo "Workspace non construit : $ESPACE_TRAVAIL/install/setup.bash est absent." >&2
        echo "Construire avec colcon build --symlink-install depuis $ESPACE_TRAVAIL." >&2
        set -u
        return 1
    fi
    source "$ESPACE_TRAVAIL/install/setup.bash"
    set -u
    export FOSA_PROFIL=simulation
    export FOSA_CONFIG_DIR="$DEPOT/caytu_config/config"
    export FOSA_CONFIG_OVERRIDE="$FOSA_CONFIG_DIR/profils/simulation.yaml"
    # Les paquets officiels (parc_msgs, referee, bringup) sont dans le même
    # espace de travail : run.sh n'a pas besoin d'un espace de base séparé.
    export FOSA_BASE_WS="$ESPACE_TRAVAIL"
    # Le profil est la source de vérité : ne jamais hériter par accident du
    # domaine ROS d'une session du robot réel ou d'une autre simulation.
    ROS_DOMAIN_ID="$(python3 -c 'from caytu_config import load_config; print(load_config().system.ros_domain_id)')"
    export ROS_DOMAIN_ID
}

graine() {
    local demandee="${1:-}"
    if [[ -n "$demandee" ]]; then
        echo "$demandee"
    elif [[ -f "$CACHE/graine" ]]; then
        cat "$CACHE/graine"
    else
        echo "$GRAINE_DEFAUT"
    fi
}

verite() {
    echo "$MONDES/parc_random_seed$1.json"
}

gazebo_actif() {
    [[ "$FOSA_SIM_GUI" == 1 ]] && echo true || echo false
}

paquet_absent() {
    ! ros2 pkg prefix "$1" >/dev/null 2>&1
}

verifier_paquets() {
    local paquets_officiels=() paquets_systeme=()
    paquet_absent parc_msgs && paquets_officiels+=(parc_msgs)
    paquet_absent parc_referee && paquets_officiels+=(parc_referee)
    paquet_absent parc_robot_bringup && paquets_officiels+=(parc_robot_bringup)
    paquet_absent depthimage_to_laserscan && paquets_systeme+=(ros-jazzy-depthimage-to-laserscan)
    if [[ "$FOSA_SIM_AUDIO" == 1 || "${FOSA_SIM_VOIX:-0}" == 1 ]]; then
        paquet_absent audio_common_msgs && paquets_systeme+=(ros-jazzy-audio-common-msgs)
    fi
    if (( ${#paquets_officiels[@]} || ${#paquets_systeme[@]} )); then
        if (( ${#paquets_officiels[@]} )); then
            echo "Paquets officiels absents : ${paquets_officiels[*]}" >&2
            echo "Cloner PARC2026-Engineers-League dans le workspace, puis le construire." >&2
        fi
        if (( ${#paquets_systeme[@]} )); then
            echo "Paquets ROS système absents : ${paquets_systeme[*]}" >&2
            printf 'Installer : sudo apt install -y %s\n' "${paquets_systeme[*]}" >&2
        fi
        echo "Puis relancer : simulation/simulation.sh preparer" >&2
        exit 1
    fi
}

preparer() {
    sourcer || exit 1
    if ! command -v colcon >/dev/null 2>&1; then
        echo "colcon introuvable : source /opt/ros/jazzy/setup.bash" >&2
        exit 1
    fi
    (
        cd "$ESPACE_TRAVAIL" || exit 1
        # Pas de --symlink-install : l'espace de travail existant a été
        # construit en copie, et mélanger les deux modes casse ament_python
        # (« existing path cannot be removed »).
        colcon build \
            --packages-select parc_msgs parc_referee parc_robot_bringup \
            parc_robot_description caytu_config caytu_interfaces caytu_perception \
            caytu_nav_solution caytu_nav_bringup caytu_course \
            --event-handlers console_cohesion-
    ) || exit 1
    mkdir -p "$JOURNAUX" "$MONDES"
    verifier_paquets
    echo "Simulation prête dans $CACHE (graine par défaut : $(graine))."
}

voix_preparer() {
    sourcer || exit 1
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
    if ros2 pkg prefix audio_common_msgs >/dev/null 2>&1; then
        echo "Départ vocal prêt. Micros disponibles : arecord -L (choix : FOSA_SIM_MICRO)."
    else
        echo "Il manque un paquet système :  sudo apt install ros-jazzy-audio-common-msgs" >&2
        exit 1
    fi
}

verifier_preparation() {
    local dossier
    dossier="$(python3 -c 'from caytu_config import load_config; print(load_config().course.preparation.directory)')"
    dossier="${dossier/#\~/$HOME}"
    if [[ ! -f "$dossier/carte.posegraph" || ! -f "$dossier/carte.data" ]]; then
        echo "Pas de préparation pour cette arène : simulation/simulation.sh preparation $(graine)" >&2
        exit 1
    fi
}

# Une course ou un Gazebo restés d'un essai précédent pilotent le même robot
# simulé et faussent tout : on refuse de démarrer.
verifier_rien_ne_tourne() {
    if pgrep -f "[p]hase2_hard.launch.py|[g]z sim|[c]ourse.launch.py|[r]eferee.launch.py|[r]espeaker_sim.py" >/dev/null; then
        echo "Une course ou Gazebo tourne déjà : simulation/simulation.sh arreter" >&2
        exit 1
    fi
}

arreter_groupe() {
    if [[ -n "${1:-}" ]]; then kill -INT -- "-$1" 2>/dev/null; fi
}

arreter() {
    pkill -INT -f "[c]ourse.launch.py"
    pkill -INT -f "[p]hase2_hard.launch.py"
    pkill -INT -f "[r]eferee.launch.py"
    pkill -INT -f "[m]oniteur.py"
    sleep 8
    # Lanceurs d'abord : ils relanceraient les nœuds de perception (respawn).
    pkill -9 -f "[c]ourse.launch.py"
    pkill -9 -f "[p]hase2_hard.launch.py"
    pkill -9 -f "[r]eferee.launch.py"
    local noeud
    for noeud in course_parc mot_depart tag_detector scan_profondeur scan_filter scan_merger \
        respeaker_node depth_converter scan_to_scan_filter_chain \
        slam_toolbox controller_server planner_server behavior_server bt_navigator \
        lifecycle_manager robot_state_publisher parameter_bridge depthimage_to_laserscan \
        referee "gz sim"; do
        pkill -9 -f "[${noeud:0:1}]${noeud:1}"
    done
    echo "Simulation arrêtée."
}

attendre_topic() {
    local topic="$1" limite="${2:-120}" fin=$((SECONDS + ${2:-120})) restant
    # `ros2 topic echo --once` quitte immédiatement si le topic n'est pas
    # encore découvert. Attendre d'abord son apparition dans le graphe ROS.
    while (( SECONDS < fin )); do
        if timeout 5 ros2 topic list 2>/dev/null | grep -Fx "$topic" >/dev/null; then
            restant=$((fin - SECONDS))
            (( restant > 0 )) || break
            if timeout "$restant" ros2 topic echo --once "$topic" >/dev/null 2>&1; then
                return 0
            fi
        fi
        sleep 1
    done
    return 1
}

attendre_texte() {
    local fichier="$1" motif="$2" limite="${3:-180}" debut=$SECONDS
    while (( SECONDS - debut < limite )); do
        grep -q "$motif" "$fichier" 2>/dev/null && return 0
        sleep 1
    done
    return 1
}

armer() {
    local ordre="${1:-}"
    local corps="{\"team\":\"Fosa simulation\""
    [[ -n "$ordre" ]] && corps+=",\"order\":\"${ordre//,/ }\""
    corps+="}"
    curl -fsS -X POST -H 'Content-Type: application/json' \
        -H "X-Judge-Token: $JETON_ARBITRE" \
        -d "$corps" "http://127.0.0.1:8080/api/judge/arm" || return 1
    echo
}

# Une phase complète dans une arène neuve (robot à la position de départ) :
# Gazebo, arbitre, course (match ou préparation), puis le moniteur au premier
# plan jusqu'au résumé final.
phase() {
    local mode="$1" ordre="${2:-}" graine_phase="$3"
    verifier_rien_ne_tourne
    mkdir -p "$JOURNAUX" "$MONDES"
    echo "$graine_phase" > "$CACHE/graine"
    local monde="$JOURNAUX/monde_$mode.log" course="$JOURNAUX/course_$mode.log"
    local arbitre="$JOURNAUX/arbitre_$mode.log" moniteur="$JOURNAUX/moniteur_$mode.log"
    local json; json="$(verite "$graine_phase")"
    echo "=== ${mode^^} (graine $graine_phase, ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
    echo "Partition Gazebo : $GZ_PARTITION"
    echo "Journaux : $JOURNAUX"

    local audio="$FOSA_SIM_AUDIO"
    [[ "$mode" == course && "${FOSA_SIM_VOIX:-0}" == 1 ]] && audio=1
    setsid ros2 launch parc_robot_bringup phase2_hard.launch.py \
        seed:="$graine_phase" gui:="$(gazebo_actif)" rviz:=false \
        audio:="$([[ "$audio" == 1 ]] && echo true || echo false)" \
        audio_device:="$FOSA_SIM_MICRO" world_dir:="$MONDES" \
        > "$monde" 2>&1 &
    local pid_monde=$!
    echo "Démarrage de Gazebo (arène seed $graine_phase)..."
    if ! attendre_topic /botwheel_explorer/odom 150; then
        echo "Le robot simulé n'a pas démarré : voir $monde" >&2
        tail -20 "$monde" >&2
        arreter_groupe "$pid_monde"; arreter
        return 1
    fi
    echo "Robot simulé prêt ; vérité terrain : $json"

    local pid_arbitre=""
    if [[ "$mode" == course ]]; then
        setsid ros2 launch parc_referee referee.launch.py \
            odom_topic:=/botwheel_explorer/odom record:=false \
            judge_token:="$JETON_ARBITRE" > "$arbitre" 2>&1 &
        pid_arbitre=$!
        if ! attendre_topic /parc/run_status 30; then
            echo "L'arbitre n'a pas démarré : voir $arbitre" >&2
            tail -20 "$arbitre" >&2
            arreter_groupe "$pid_monde"; arreter_groupe "$pid_arbitre"; arreter
            return 1
        fi
        echo "Arbitre : http://localhost:8080/  (juges : /judge?token=$JETON_ARBITRE)"
    fi

    local arguments=()
    [[ "$mode" == preparation ]] && arguments=(mode:=preparation)
    setsid "$DEPOT/run.sh" course-premier-plan "${arguments[@]}" > "$course" 2>&1 &
    local pid_course=$!

    if [[ "$mode" == course ]]; then
        if ! attendre_texte "$course" 'ATTENTE' 180; then
            echo "La course n'est pas prête : voir $course" >&2
            tail -20 "$course" >&2
            arreter_groupe "$pid_course"; arreter_groupe "$pid_arbitre"; arreter_groupe "$pid_monde"; arreter
            return 1
        fi
        if ! armer "$ordre"; then
            echo "Armement de l'arbitre impossible (port 8080 occupé ?)." >&2
            tail -20 "$arbitre" >&2
            arreter_groupe "$pid_course"; arreter_groupe "$pid_arbitre"; arreter_groupe "$pid_monde"; arreter
            return 1
        fi
        sleep 3
        if [[ "${FOSA_SIM_VOIX:-0}" == 1 ]]; then
            echo ">>> Départ À LA VOIX : dites « allez » distinctement dans le micro."
            echo ">>> (l'arbitre est armé ; la course attend le mot de départ)"
        else
            (cd "$DEPOT" && timeout 20 ./run.sh depart > /dev/null 2>&1)
        fi
    fi

    FOSA_SIM_VERITE="$json" timeout "$DUREE_MAX" \
        python3 -u "$DOSSIER_SIM/moniteur.py" --quitter-a-la-fin \
        | tee "$moniteur"
    echo "Arrêt de la phase ${mode}..."
    arreter_groupe "$pid_course"
    sleep 3
    arreter_groupe "$pid_arbitre"
    arreter_groupe "$pid_monde"
    # Les nœuds relancés par respawn survivent au lanceur : arrêt complet.
    arreter > /dev/null
}

case "${1:-}" in
    preparer) preparer ;;
    voix-preparer) voix_preparer ;;
    arreter) arreter ;;
    micro)
        sourcer || exit 1
        exec ros2 run parc_robot_bringup respeaker_sim.py --ros-args \
            -p source:=mic -p device:="$FOSA_SIM_MICRO" ;;
    essai)
        sourcer || exit 1; preparer
        graine_essai="$(graine "${3:-}")"
        phase preparation "" "$graine_essai" && phase course "${2:-}" "$graine_essai"
        ;;
    preparation)
        sourcer || exit 1; preparer; graine_phase="$(graine "${2:-}")"
        phase preparation "" "$graine_phase"
        ;;
    course)
        sourcer || exit 1; verifier_preparation
        graine_course="$(graine "${3:-}")"
        if [[ -f "$CACHE/graine" && "$(cat "$CACHE/graine")" != "$graine_course" ]]; then
            echo "Préparation faite avec la graine $(cat "$CACHE/graine"), pas $graine_course." >&2
            echo "Relancer : simulation/simulation.sh preparation $graine_course" >&2
            exit 1
        fi
        phase course "${2:-}" "$graine_course"
        ;;
    monde)
        sourcer || exit 1; verifier_paquets
        graine_monde="$(graine "${2:-}")"
        mkdir -p "$MONDES" "$CACHE"
        echo "$graine_monde" > "$CACHE/graine"
        exec ros2 launch parc_robot_bringup phase2_hard.launch.py \
            seed:="$graine_monde" gui:="$(gazebo_actif)" rviz:="${FOSA_SIM_RVIZ:-0}" \
            audio:="$FOSA_SIM_AUDIO" audio_device:="$FOSA_SIM_MICRO" world_dir:="$MONDES"
        ;;
    moniteur)
        sourcer || exit 1
        graine_monde="$(graine "${2:-}")"
        json="$(verite "$graine_monde")"
        [[ -f "$json" ]] || json="$(ls -t "$MONDES"/parc_random_seed*.json 2>/dev/null | head -1)"
        [[ -n "$json" && -f "$json" ]] || { echo "Aucune vérité terrain dans $MONDES" >&2; exit 1; }
        FOSA_SIM_VERITE="$json" exec python3 -u "$DOSSIER_SIM/moniteur.py"
        ;;
    *) sed -n '2,20p' "$0"; exit 2 ;;
esac
