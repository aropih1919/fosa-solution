#!/usr/bin/env bash
# Diagnostic du robot réel (premier créneau d'essai) : vérifie que les
# interfaces attendues par caytu_config existent, mesure caméras/audio/TF et,
# sur demande, enregistre un rosbag pour régler la solution hors du robot.
#
# Usage (après sourcing, normalement via ./run.sh diagnostic-robot) :
#   outils/diagnostic_robot.sh                 # vérifications (~1 min)
#   outils/diagnostic_robot.sh --enregistrer 120   # + rosbag de 120 s
set -uo pipefail

DUREE_BAG=0
if [[ "${1:-}" == "--enregistrer" ]]; then
    DUREE_BAG="${2:-120}"
fi
DONNEES="${FOSA_DONNEES:-$HOME/.ros/fosa}"
mkdir -p "$DONNEES"
RAPPORT="$DONNEES/diagnostic_$(date +%Y%m%d_%H%M%S).txt"
exec > >(tee "$RAPPORT") 2>&1

ok() { printf '  [OK]   %s\n' "$*"; }
ko() { printf '  [ÉCHEC] %s\n' "$*"; }
info() { printf '  [INFO] %s\n' "$*"; }
titre() { printf '\n== %s\n' "$*"; }
lire_config() {
    python3 -c "from caytu_config import load_config; c = load_config(); print($1)"
}

titre "Environnement"
info "ROS_DISTRO=${ROS_DISTRO:-?} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-?} RMW=${RMW_IMPLEMENTATION:-défaut}"
info "OpenCV : $(python3 -c 'import cv2; print(cv2.__version__)' 2>&1)"
python3 -c 'import vosk' 2>/dev/null && ok "Vosk importable" || ko "Vosk absent (départ vocal impossible)"
for paquet in nav2_bt_navigator nav2_controller nav2_smac_planner \
    nav2_regulated_pure_pursuit_controller slam_toolbox caytu_course; do
    ros2 pkg prefix "$paquet" >/dev/null 2>&1 && ok "paquet $paquet" || ko "paquet $paquet ABSENT"
done
MODELE="$(lire_config 'c.start_word.model_path')"
CONFIG_DIR="${FOSA_CONFIG_DIR:-$(cd "$(dirname "$0")/../caytu_config/config" && pwd)}"
[[ "$MODELE" = /* ]] || MODELE="$CONFIG_DIR/$MODELE"
[[ -d "$MODELE" ]] && ok "modèle Vosk : $MODELE" || ko "modèle Vosk absent : $MODELE"

titre "Topics attendus par la configuration"
LISTE="$(ros2 topic list -t 2>/dev/null)"
verifier_topic() {
    local ligne
    ligne="$(grep -E "^$1 " <<<"$LISTE" || true)"
    if [[ -n "$ligne" ]]; then ok "$ligne"; else ko "$1 absent ($2)"; fi
}
verifier_topic "$(lire_config c.navigation.scan_topic)" "scan SLAM/costmaps"
verifier_topic "$(lire_config c.navigation.odom_topic)" "odométrie"
verifier_topic "$(lire_config c.navigation.cmd_vel_topic)" "commande (doit être geometry_msgs/msg/Twist)"
verifier_topic "$(lire_config c.start_word.audio_topic)" "audio du mot de départ"
verifier_topic "$(lire_config c.course.order_topic)" "ordre des organisateurs (souvent publié au départ seulement)"
for camera in $(lire_config "' '.join(x.color_topic + ',' + x.info_topic for x in c.perception.apriltag.cameras)"); do
    verifier_topic "${camera%%,*}" "image couleur"
    verifier_topic "${camera##*,}" "calibration"
done
for source in $(lire_config "' '.join(s.topic for s in c.navigation.obstacle_sources)"); do
    verifier_topic "$source" "source d'obstacles (activée ou non dans fosa.yaml)"
done

titre "Type de la commande de vitesse"
ros2 topic info "$(lire_config c.navigation.cmd_vel_topic)" 2>&1 | sed 's/^/  /'
info "Nav2 publie un Twist non horodaté (enable_stamped_cmd_vel: false)."

titre "Mesures (caméras, scans, repères, audio)"
python3 "$(dirname "$0")/mesures_robot.py"

titre "Un message de chaque capteur"
for topic in "$(lire_config c.navigation.odom_topic)" "$(lire_config c.navigation.scan_topic)"; do
    if timeout 5 ros2 topic echo --once "$topic" >/dev/null 2>&1; then
        ok "$topic publie"
    else
        ko "$topic ne publie rien en 5 s"
    fi
done

if [[ "$DUREE_BAG" -gt 0 ]]; then
    titre "Enregistrement rosbag ($DUREE_BAG s)"
    if ros2 bag --help >/dev/null 2>&1; then
        SORTIE="$DONNEES/bags/essai_$(date +%H%M%S)"
        mkdir -p "$DONNEES/bags"
        CAMERAS="$(lire_config "' '.join(x.color_topic + ' ' + x.info_topic for x in c.perception.apriltag.cameras)")"
        # shellcheck disable=SC2086
        timeout -s INT "$DUREE_BAG" ros2 bag record -o "$SORTIE" /tf /tf_static \
            "$(lire_config c.navigation.scan_topic)" /scan \
            "$(lire_config c.navigation.odom_topic)" \
            /camera1/camera1/imu /camera2/camera2/imu /audio /speech_audio \
            $CAMERAS || true
        ok "rosbag : $SORTIE"
    else
        ko "ros2 bag indisponible sur ce poste"
    fi
fi

titre "Rapport enregistré : $RAPPORT"
