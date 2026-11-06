#!/usr/bin/env bash
# Répétition PC sur le simulateur officiel, avec arrêt limité à nos sessions.
set -euo pipefail
DOSSIER_SIM="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPOT="$(cd -- "$DOSSIER_SIM/.." && pwd)"
ESPACE_TRAVAIL="${FOSA_WS:-$(cd -- "$DEPOT/../.." && pwd)}"
source "$DEPOT/outils/config_exploitation.sh"
CACHE="${FOSA_SIM_CACHE:-$(lire_exploitation simulation.cache)}"
CACHE="${CACHE/#\~/$HOME}"
JOURNAUX="$CACHE/journaux/$(date +%Y%m%d_%H%M%S)_$$"
MONDES="$CACHE/mondes"
export FOSA_PROFIL=simulation FOSA_DONNEES="$CACHE"
export FOSA_WS="$ESPACE_TRAVAIL" FOSA_BASE_WS="$ESPACE_TRAVAIL"
export GZ_PARTITION="${FOSA_SIM_GZ_PARTITION:-fosa_simulation_${UID}_$$}"
DUREE_MAX="${FOSA_SIM_DUREE_MAX:-$(lire_exploitation simulation.duree_max_sec)}"
JETON="$(lire_exploitation simulation.jeton_arbitre)"
PORT="${FOSA_SIM_PORT:-$(lire_exploitation simulation.port_arbitre)}"
MICRO="${FOSA_SIM_MICRO:-$(lire_exploitation simulation.micro)}"
PROCESSUS="$DEPOT/outils/processus.py"

aide() {
    cat <<'AIDE'
Usage : simulation/simulation.sh MODE [ordre] [graine]
  preparer                     compile les six paquets Fosa et les quatre paquets officiels
  course-sans-carte [ordre] [graine]  match autonome sans SLAM ni préparation
  essai [ordre] [graine]        préparation SLAM puis match sur deux mondes neufs
  preparation [graine]         cartographie autonome et mémoire des tags
  course [ordre] [graine]      match sur la préparation de cette graine
  monde [graine]              monde seul pour une préparation guidée
  voix-preparer               Vosk et modèle pour le micro du PC
  micro                      publie le micro du PC (simulateur seul)
  arreter                    arrête uniquement les processus de nos sessions

Ordre de répétition par défaut : 4,9,1,7 (annonce du 06/11).
FOSA_SIM_GUI=0 : sans fenêtre. FOSA_SIM_VOIX=1 : attendre « allez ».
Le match dure au plus 600 secondes RÉELLES, comme l'arbitre officiel.
AIDE
}

sourcer() {
    set +u
    source "/opt/ros/$(lire_exploitation ros_distro)/setup.bash"
    if [[ -f "$ESPACE_TRAVAIL/install/setup.bash" ]]; then
        source "$ESPACE_TRAVAIL/install/setup.bash"
    fi
    if [[ -d "$ESPACE_TRAVAIL/.fosa_python" ]]; then
        export PYTHONPATH="$ESPACE_TRAVAIL/.fosa_python${PYTHONPATH:+:$PYTHONPATH}"
    fi
    set -u
    export FOSA_CONFIG_DIR="$DEPOT/caytu_config/config"
    export FOSA_CONFIG_OVERRIDE="$FOSA_CONFIG_DIR/profils/simulation.yaml"
    export PYTHONPATH="$DEPOT/caytu_config${PYTHONPATH:+:$PYTHONPATH}"
    ROS_DOMAIN_ID="$(python3 -c 'from caytu_config import load_config; print(load_config().system.ros_domain_id)')"
    export ROS_DOMAIN_ID
    source "$DEPOT/outils/environnement_dds.sh"
    configurer_dds
}

preparer() {
    sourcer
    local officiels="${FOSA_PARC_SRC:-$ESPACE_TRAVAIL/src}" paquet chemins=("$DEPOT") paquets=()
    mapfile -t paquets < <(lire_exploitation paquets_fosa)
    for paquet in parc_msgs parc_referee parc_robot_bringup parc_robot_description; do
        [[ -f "$officiels/$paquet/package.xml" ]] || {
            echo "Paquet officiel absent : $officiels/$paquet (FOSA_PARC_SRC)." >&2; return 1;
        }
        chemins+=("$officiels/$paquet"); paquets+=("$paquet")
    done
    (cd "$ESPACE_TRAVAIL" && colcon build --base-paths "${chemins[@]}" \
        --packages-select "${paquets[@]}" --executor sequential)
    echo "Workspace construit : $ESPACE_TRAVAIL"
}

verifier_paquets() {
    local paquet
    for paquet in parc_msgs parc_referee parc_robot_bringup caytu_course \
        depthimage_to_laserscan ros_gz_bridge; do
        ros2 pkg prefix "$paquet" >/dev/null || return 1
    done
}

actif() { python3 "$PROCESSUS" actif --etat "$CACHE/sessions/$1.json"; }
lancer() {
    local nom="$1"; shift
    python3 "$PROCESSUS" lancer --etat "$CACHE/sessions/$nom.json" \
        --journal "$JOURNAUX/$nom.log" -- "$@"
}
arreter() {
    local nom
    for nom in course moniteur arbitre monde pont_pose; do
        python3 "$PROCESSUS" arreter --etat "$CACHE/sessions/$nom.json"
    done
}

demarrer_pont_pose() {
    # Le PosePublisher de Gazebo Jazzy publie /model/sitoe_robot/pose ; le pont
    # officiel attend sitoe_robot/pose et laisse donc /sim/pose vide. On ponte
    # le vrai topic vers le topic de vérité du moniteur, sans modifier le
    # paquet officiel.
    local gz_topic ros_topic
    gz_topic="$(lire_exploitation simulation.gz_pose_topic)"
    ros_topic="$(lire_exploitation simulation.jugement.topic_pose)"
    lancer pont_pose ros2 run ros_gz_bridge parameter_bridge \
        "${gz_topic}@geometry_msgs/msg/Pose[gz.msgs.Pose" \
        --ros-args -r "${gz_topic}:=${ros_topic}"
}

attendre_topic() {
    local topic="$1" limite="$2"
    # Découverte et réception réelles, compatibles avec les capteurs best effort.
    timeout "$limite" python3 - "$topic" <<'PYTHON'
import sys
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
rclpy.init()
noeud = rclpy.create_node('attente_capteur_fosa')
recu = []
abonnement = None
while rclpy.ok() and not recu:
    if abonnement is None:
        types = dict(noeud.get_topic_names_and_types()).get(sys.argv[1])
        if types:
            abonnement = noeud.create_subscription(get_message(types[0]), sys.argv[1],
                                                   recu.append, qos_profile_sensor_data)
    rclpy.spin_once(noeud, timeout_sec=0.1)
noeud.destroy_node()
rclpy.shutdown()
PYTHON
}

attendre_course() {
    local fin=$((SECONDS + $(lire_exploitation simulation.delai_course_sec)))
    while ((SECONDS < fin)); do
        grep -q 'ATTENTE' "$JOURNAUX/course.log" && return 0
        actif course || break
        sleep 1
    done
    echo "Course indisponible : $JOURNAUX/course.log" >&2
    return 1
}

demarrer_monde() {
    local graine="$1" audio=false gui=true
    [[ "${FOSA_SIM_GUI:-1}" == 0 ]] && gui=false
    [[ "${FOSA_SIM_VOIX:-0}" == 1 || "${FOSA_SIM_AUDIO:-0}" == 1 ]] && audio=true
    mkdir -p "$MONDES" "$JOURNAUX"
    lancer monde ros2 launch parc_robot_bringup phase2_hard.launch.py \
        seed:="$graine" gui:="$gui" rviz:=false audio:="$audio" \
        audio_device:="$MICRO" world_dir:="$MONDES"
    attendre_topic /botwheel_explorer/odom "$(lire_exploitation simulation.delai_robot_sec)"
}

phase() {
    local mode="$1" ordre="$2" graine="$3" resultat=0 arguments=()
    local JOURNAUX="$JOURNAUX/$mode"
    demarrer_monde "$graine"
    demarrer_pont_pose
    if [[ "$mode" == course ]]; then
        lancer arbitre ros2 launch parc_referee referee.launch.py \
            odom_topic:=/botwheel_explorer/odom record:=false web_port:="$PORT" judge_token:="$JETON"
        attendre_topic /parc/run_status "$(lire_exploitation simulation.delai_arbitre_sec)"
    else
        arguments=(mode:=preparation)
    fi
    local options_moniteur=()
    [[ "$mode" != course ]] || options_moniteur=(--exiger-visites --port-arbitre "$PORT")
    lancer moniteur timeout "$DUREE_MAX" python3 -u "$DOSSIER_SIM/moniteur.py" \
        --verite "$MONDES/parc_random_seed$graine.json" --quitter-a-la-fin \
        --pret "$JOURNAUX/moniteur.pret" --rapport "$JOURNAUX/rapport_$mode.json" \
        "${options_moniteur[@]}"
    local fin=$((SECONDS + $(lire_exploitation simulation.delai_robot_sec)))
    while [[ ! -f "$JOURNAUX/moniteur.pret" ]]; do
        if ! actif moniteur || ((SECONDS >= fin)); then
            echo "Moniteur indisponible : $JOURNAUX/moniteur.log" >&2; return 1
        fi
        sleep 1
    done
    lancer course "$DEPOT/run.sh" course-premier-plan "${arguments[@]}"
    if [[ "$mode" == course ]]; then
        attendre_course
        local corps
        corps="$(python3 - "$ordre" <<'PYTHON'
import json
import sys
from caytu_config import load_config
from caytu_course.ordre_tags import lire_ordre
ordre = lire_ordre(sys.argv[1], load_config().course.tag_ids)
print(json.dumps({'team': 'Fosa simulation', 'order': ' '.join(map(str, ordre))}))
PYTHON
)"
        curl -fsS -X POST -H 'Content-Type: application/json' -H "X-Judge-Token: $JETON" \
            -d "$corps" "http://127.0.0.1:$PORT/api/judge/arm"
        if [[ "${FOSA_SIM_VOIX:-0}" == 1 ]]; then
            echo 'Arbitre armé : attendez ATTENTE_DEPART, puis dites « allez ».'
        else
            "$DEPOT/run.sh" depart
        fi
    fi
    echo "Suivi : tail -f $JOURNAUX/moniteur.log"
    while actif moniteur; do sleep 1; done
    python3 - "$JOURNAUX/rapport_$mode.json" <<'PYTHON' || resultat=$?
import json
import sys
from pathlib import Path
chemin = Path(sys.argv[1])
if not chemin.is_file():
    sys.exit('Aucun rapport : délai dépassé ou moniteur interrompu.')
rapport = json.loads(chemin.read_text())
print(json.dumps(rapport, ensure_ascii=False, indent=2))
sys.exit(0 if rapport['reussi'] else 1)
PYTHON
    arreter
    if [[ "$mode" == preparation && "$resultat" == 0 ]]; then
        python3 -c 'from caytu_config import load_config; from caytu_course.lancement import carte_preparee; assert carte_preparee(load_config()) is not None, "Carte non enregistrée"' || return 1
        echo "$graine" > "$CACHE/graine_preparation"
    fi
    echo "Résultat $mode : code $resultat ; journaux $JOURNAUX"
    return "$resultat"
}

mode="${1:---help}"
case "$mode" in
    --help|-h|help) aide; exit 0 ;;
    arreter) arreter; exit 0 ;;
    preparer) preparer; exit 0 ;;
    voix-preparer)
        sourcer
        modele="$(lire_exploitation modele_vosk)"
        python3 -m pip install --target "$ESPACE_TRAVAIL/.fosa_python" vosk==0.3.45
        mkdir -p "$CACHE" "$DEPOT/caytu_config/config/models"
        curl -fL "$(lire_exploitation url_modele_vosk)" -o "$CACHE/$modele.zip"
        python3 -m zipfile -e "$CACHE/$modele.zip" "$DEPOT/caytu_config/config/models"
        exit 0 ;;
    micro)
        sourcer
        exec ros2 run parc_robot_bringup respeaker_sim.py --ros-args -p source:=mic -p device:="$MICRO" ;;
    course-sans-carte) export FOSA_NAVIGATION=sans_carte ;;
    essai|preparation|course|monde) export FOSA_NAVIGATION=carte ;;
    *) aide >&2; exit 2 ;;
esac
sourcer
verifier_paquets
mkdir -p "$CACHE/sessions"
exec 9>"$CACHE/scenario.lock"
flock -n 9 || { echo 'Une répétition Fosa tourne déjà.' >&2; exit 1; }
for nom in monde arbitre course moniteur pont_pose; do
    if actif "$nom"; then
        echo "Session $nom active : simulation/simulation.sh arreter" >&2; exit 1
    fi
done
trap arreter EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ordre="${2:-$(lire_exploitation simulation.ordre_repetition)}"
graine="${3:-$(lire_exploitation simulation.graine)}"
if [[ "$mode" == preparation || "$mode" == monde ]]; then
    graine="${2:-$(lire_exploitation simulation.graine)}"
fi
[[ "$graine" =~ ^[0-9]+$ ]] || { echo 'Graine entière positive attendue.' >&2; exit 2; }
case "$mode" in
    course-sans-carte) phase course "$ordre" "$graine" ;;
    preparation) phase preparation '' "$graine" ;;
    essai) phase preparation '' "$graine"; phase course "$ordre" "$graine" ;;
    course)
        [[ -f "$CACHE/graine_preparation" && "$(cat "$CACHE/graine_preparation")" == "$graine" ]] || {
            echo 'Préparation absente ou graine différente. Utilisez preparation ou course-sans-carte.' >&2; exit 1;
        }
        phase course "$ordre" "$graine" ;;
    monde)
        demarrer_monde "$graine"
        echo "$graine" > "$CACHE/graine_preparation"
        tail -f "$JOURNAUX/monde.log" ;;
esac
