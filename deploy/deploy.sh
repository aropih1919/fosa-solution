#!/usr/bin/env bash
# Début de créneau : envoie la solution sur le robot et la compile.
#
#   deploy/deploy.sh                    code seulement (cartographie)
#   deploy/deploy.sh --map arene_dakar  code + une carte de robot_data/maps/
#   options, cumulables :
#     --installer   lance installer_robot.sh sur le robot (vérifie, installe ce
#                   qui manque après confirmation) avant la compilation
#     --paquets     envoie aussi faster-whisper et le modèle Whisper téléchargés
#                   sur le portable par deploy/telecharger_pour_robot.sh (robot
#                   sans Internet) ; avec --installer, ils sont installés
#     --parc-msgs   envoie aussi parc_msgs du dépôt PARC du portable, SEULEMENT
#                   si le robot ne l'a pas (normalement dans ~/ros2_ws)
#
# Partent les sept paquets ROS de la solution (cartographie, tags, course,
# mot de départ), sans tests, docs, outils ni modèles d'IA : rien de plus que ce
# qu'il faut pour tourner. Tout va dans $ROBOT_DIR (dossier d'équipe, mémoire
# vive par défaut) : effacé par clean.sh. Durée : 1 à 3 min (+ installation).
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

MAP=""; INSTALLER=0; PAQUETS_LOCAUX=0; PARC_MSGS=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --map) MAP="${2:?--map demande un nom de carte}"; shift 2 ;;
        --installer) INSTALLER=1; shift ;;
        --paquets) PAQUETS_LOCAUX=1; shift ;;
        --parc-msgs) PARC_MSGS=1; shift ;;
        -h|--help) sed -n '2,/^[^#]/{/^#/s/^# \{0,1\}//p}' "$0"; exit 0 ;;
        *) echo "Option inconnue : $1 (voir --help)" >&2; exit 1 ;;
    esac
done
if [[ -n "$MAP" && ! -f "$LOCAL_DATA/maps/$MAP/fosa_map.yaml" ]]; then
    echo "Carte $LOCAL_DATA/maps/$MAP introuvable (récupérée avec deploy/fetch.sh ?)." >&2
    exit 1
fi
if (( PAQUETS_LOCAUX )) && [[ ! -d "$LOCAL_DATA/paquets/roues" ]]; then
    echo "$LOCAL_DATA/paquets absent : lancer d'abord deploy/telecharger_pour_robot.sh." >&2
    exit 1
fi
SOURCE_PARC_MSGS=""
if (( PARC_MSGS )); then
    # Dépôt PARC du portable : à côté de fosa-solution (install.sh), ou FOSA_PARC_DEPOT.
    DEPOT_PARC="${FOSA_PARC_DEPOT:-$(dirname "$REPO_DIR")}"
    SOURCE_PARC_MSGS="$(dirname "$(grep -Rls --include=package.xml '<name>parc_msgs</name>' \
        "$DEPOT_PARC" 2> /dev/null | head -n 1)")"
    if [[ ! -f "$SOURCE_PARC_MSGS/package.xml" ]]; then
        echo "parc_msgs introuvable dans $DEPOT_PARC (dépôt PARC2026-Engineers-League cloné ?" \
             "sinon FOSA_PARC_DEPOT=<dossier du dépôt>)." >&2
        exit 1
    fi
fi

echo "== Préparation de $ROBOT_DIR sur $ROBOT"
robot_ssh bash -s -- "$ROBOT_DIR" "$ROBOT_BASE_SETUP" <<'REMOTE'
set -euo pipefail
ROOT="$1"; BASE="$2"
[[ -f "$BASE" ]] || { echo "Espace de travail de base introuvable : $BASE" >&2; exit 1; }
mkdir -p "$ROOT"/{ws/src,maps,tmp,ros_log,py,hf,paquets}
chmod 700 "$ROOT"
# Environnement de nos terminaux sur le robot : rien ne sort de $ROOT.
cat > "$ROOT/env.sh" <<EOF
# Environnement Fosa sur le robot (écrit par deploy.sh).
[ -f ~/.bashrc ] && source ~/.bashrc          # réglages ROS des organisateurs
unset HISTFILE; export HISTFILE=/dev/null     # pas d'historique dans le compte partagé
export FOSA_ROOT="$ROOT"
export FOSA_MAPS_DIR="$ROOT/maps"             # cartes et enregistrements
export ROS_LOG_DIR="$ROOT/ros_log"            # journaux ROS hors de ~/.ros
export TMPDIR="$ROOT/tmp"                     # fichiers de paramètres temporaires
# Mot de départ : faster-whisper et son modèle, s'il faut les installer, vont
# aussi dans notre dossier (bash \$FOSA_ROOT/installer_robot.sh).
export HF_HOME="$ROOT/hf"
# Modèle Whisper déjà là (installer_robot.sh ou --paquets) : aucun accès réseau
# au chargement, même sans Internet.
if compgen -G "\$HF_HOME/hub/models--*faster-whisper-*" > /dev/null; then export HF_HUB_OFFLINE=1; fi
# caytu_config du robot réel (temps réel, /cmd_vel, caméras du robot).
export FOSA_CONFIG_OVERRIDE="$ROOT/ws/src/caytu_config/config/robot_reel.yaml"
source "$BASE"
[ -f "$ROOT/ws/install/setup.bash" ] && source "$ROOT/ws/install/setup.bash"
export PYTHONPATH="$ROOT/py\${PYTHONPATH:+:\$PYTHONPATH}"
# CycloneDDS des organisateurs avec ParticipantIndex « auto » : 10 processus
# ROS au plus par défaut (MaxAutoParticipantIndex 9). Le robot en fait déjà
# tourner beaucoup ; au-delà, « Failed to find a free participant index ». On
# relève la limite pour NOS processus seulement, sans toucher à leur fichier.
case "\${CYCLONEDDS_URI:-}" in
  *MaxAutoParticipantIndex*) ;;
  *) [ "\${RMW_IMPLEMENTATION:-}" = rmw_cyclonedds_cpp ] || [ -n "\${CYCLONEDDS_URI:-}" ] && \
     export CYCLONEDDS_URI="\${CYCLONEDDS_URI:+\$CYCLONEDDS_URI,}<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>" ;;
esac
cd "$ROOT"
PS1="[fosa] \w\\$ "
EOF
REMOTE

# Script d'installation des paquets manquants : envoyé sans rsync, pour qu'il
# soit là même si rsync manque au robot.
robot_ssh "cat > '$ROBOT_DIR/installer_robot.sh' && chmod 700 '$ROBOT_DIR/installer_robot.sh'" \
    < "$DEPLOY_DIR/installer_robot.sh"

echo "== Envoi du code"
# Course complète : configuration, messages, navigation, détection et registre
# des tags (caytu_perception), mot de départ (caytu_assistant, qui dépend de
# caytu_semantic_map). Jamais les modèles d'IA (config/models/).
PAQUETS=(caytu_config caytu_interfaces caytu_semantic_map caytu_perception
         caytu_assistant caytu_nav_bringup caytu_nav_solution)
robot_rsync --delete --exclude test/ --exclude tools/ --exclude __pycache__/ --exclude '*.pyc' \
    --exclude .pytest_cache/ --exclude '*.egg-info/' --exclude 'config/models/' \
    "${PAQUETS[@]/#/$REPO_DIR/}" "$ROBOT:$ROBOT_DIR/ws/src/" || {
    echo "Envoi impossible (rsync absent du robot ?). deploy/connect.sh puis :" >&2
    echo "  bash \$FOSA_ROOT/installer_robot.sh" >&2
    exit 1
}

if [[ -n "$SOURCE_PARC_MSGS" ]]; then
    if robot_ssh "source '$ROBOT_BASE_SETUP' > /dev/null 2>&1; ros2 pkg prefix parc_msgs" > /dev/null 2>&1; then
        echo "== parc_msgs déjà présent sur le robot : celui des organisateurs est gardé."
        robot_ssh "rm -rf '$ROBOT_DIR/ws/src/parc_msgs'"
    else
        echo "== Envoi de parc_msgs (absent du robot)"
        robot_rsync --delete --exclude .git/ "$SOURCE_PARC_MSGS/" "$ROBOT:$ROBOT_DIR/ws/src/parc_msgs/"
    fi
fi

if [[ -n "$MAP" ]]; then
    echo "== Envoi de la carte $MAP"
    robot_rsync --delete "$LOCAL_DATA/maps/$MAP" "$ROBOT:$ROBOT_DIR/maps/"
fi

if (( PAQUETS_LOCAUX )); then
    echo "== Envoi des paquets Python et du modèle Whisper ($(du -shL "$LOCAL_DATA/paquets" | cut -f1))"
    robot_rsync --delete "$LOCAL_DATA/paquets/roues/" "$ROBOT:$ROBOT_DIR/paquets/roues/"
    if [[ -d "$LOCAL_DATA/paquets/hf/hub" ]]; then
        robot_ssh "mkdir -p '$ROBOT_DIR/hf/hub'"
        robot_rsync "$LOCAL_DATA/paquets/hf/hub/" "$ROBOT:$ROBOT_DIR/hf/hub/"
    fi
fi

if (( INSTALLER )); then
    echo "== Installation de ce qui manque (installer_robot.sh, sur le robot)"
    # -t : l'installateur demande confirmation (et sudo pour apt).
    robot_ssh -t "FOSA_ROOT='$ROBOT_DIR' FOSA_DEPLOIEMENT=1 bash '$ROBOT_DIR/installer_robot.sh'" \
        || echo "installer_robot.sh signale des manques (voir ci-dessus) : on compile quand même."
fi

echo "== Compilation sur le robot"
robot_ssh bash -s -- "$ROBOT_DIR" <<'REMOTE'
set -o pipefail
source "$1/env.sh" > /dev/null    # sans -e : le ~/.bashrc du robot peut échouer sans gravité
set -e
cd "$FOSA_ROOT/ws"
command -v colcon > /dev/null || {
    echo "colcon absent du robot. deploy/connect.sh puis : bash $FOSA_ROOT/installer_robot.sh" >&2
    exit 1
}
colcon build --cmake-args -DBUILD_TESTING=OFF 2>&1 | tail -3
set +e
source install/setup.bash
# Indispensables à la course.
missing=""
for pkg in slam_toolbox nav2_bt_navigator nav2_collision_monitor laser_filters \
           pointcloud_to_laserscan depth_image_proc cv_bridge teleop_twist_keyboard; do
    ros2 pkg prefix "$pkg" > /dev/null 2>&1 || missing="$missing $pkg"
done
# Messages de l'arbitre officiel (/parc/*) : dans ~/ros2_ws des organisateurs.
python3 -c "from parc_msgs.msg import TagOrder, RunStatus, StartSignal, TagVisit, Finish" \
    > /dev/null 2>&1 || missing="$missing parc_msgs(arbitre:ROBOT_BASE_SETUP?_sinon_deploy.sh_--parc-msgs)"
python3 -c "import cv2" > /dev/null 2>&1 || missing="$missing python3:cv2(image_JPEG_pour_l_arbitre)"
# Détecteur des tags choisi dans caytu_config (apriltag_ros par défaut).
detecteur=$(python3 -c "from caytu_config import load_config; print(load_config().perception.apriltag.detector)" 2> /dev/null)
optional=""
if [[ "${detecteur:-apriltag_ros}" == apriltag_ros ]]; then
    for pkg in apriltag_ros apriltag_msgs; do
        ros2 pkg prefix "$pkg" > /dev/null 2>&1 || missing="$missing $pkg(détecteur_des_tags:ros-jazzy-apriltag-ros)"
    done
    python3 -c "import cv2; cv2.aruco.getPredefinedDictionary" > /dev/null 2>&1 \
        || optional="$optional python3:cv2.aruco(secours_detecteur:=opencv)"
else
    python3 -c "import cv2; cv2.aruco.getPredefinedDictionary" > /dev/null 2>&1 \
        || missing="$missing python3:cv2.aruco(opencv-contrib)"
fi
[[ -z "$missing" ]] || echo "INDISPENSABLE absent du robot :$missing"
# Pour la voix (voix:=true).
ros2 pkg prefix audio_common_msgs > /dev/null 2>&1 || optional="$optional audio_common_msgs(voix)"
python3 -c "import faster_whisper" > /dev/null 2>&1 || optional="$optional python3:faster_whisper(voix)"
[[ -z "$optional" ]] || echo "Optionnel absent :$optional"
[[ -z "$missing$optional" ]] || echo "  -> dans deploy/connect.sh : bash \$FOSA_ROOT/installer_robot.sh"
ros2 pkg prefix caytu_nav_bringup > /dev/null 2>&1 \
    || { echo "ÉCHEC : la compilation n'a pas produit caytu_nav_bringup." >&2; exit 1; }
REMOTE
echo "Prêt. Ouvrir un terminal sur le robot : deploy/connect.sh"
