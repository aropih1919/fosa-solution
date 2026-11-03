#!/usr/bin/env bash
# Déploiement sur le robot PARC (Jetson AGX Orin, ROS 2 Jazzy) SANS sudo.
#
# Règlement : tout dans le dossier de l'équipe, rien dans le système ; les
# paquets système (Nav2, slam_toolbox) se demandent aux organisateurs.
#
#   Sur le PC, avec Internet :  ./deployer_robot.sh --preparer
#       télécharge dans livraison/ les roues Vosk aarch64 et le modèle FR,
#       à copier avec le dépôt sur le robot (scp/rsync).
#   Sur le robot :               ./deployer_robot.sh
#       vérifie les prérequis, installe Vosk dans <workspace>/.venv,
#       décompresse le modèle et construit les paquets de la course.
set -euo pipefail

DOSSIER_SCRIPT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ESPACE_TRAVAIL="$(cd -- "$DOSSIER_SCRIPT/../.." && pwd)"
LIVRAISON="${FOSA_LIVRAISON:-$DOSSIER_SCRIPT/livraison}"
MODELE="vosk-model-small-fr-0.22"
URL_MODELE="https://alphacephei.com/vosk/models/$MODELE.zip"
DOSSIER_MODELES="$DOSSIER_SCRIPT/caytu_config/config/models"
PAQUETS_ROS=(nav2_controller nav2_planner nav2_behaviors nav2_bt_navigator
    nav2_lifecycle_manager nav2_smac_planner nav2_regulated_pure_pursuit_controller
    nav2_costmap_2d nav2_msgs slam_toolbox cv_bridge tf2_ros)

preparer() {
    mkdir -p "$LIVRAISON/roues"
    echo "[1/2] Roues Python aarch64 / Python 3.12 (Vosk et dépendances)..."
    python3 -m pip download vosk --dest "$LIVRAISON/roues" --only-binary=:all: \
        --platform manylinux2014_aarch64 --python-version 3.12
    echo "[2/2] Modèle Vosk $MODELE..."
    [[ -f "$LIVRAISON/$MODELE.zip" ]] || curl -fL "$URL_MODELE" -o "$LIVRAISON/$MODELE.zip"
    echo "Copier le dépôt (avec livraison/) dans <dossier équipe>/ws/src/ du robot."
}

verifier_prerequis() {
    local manquants=()
    [[ -f /opt/ros/jazzy/setup.bash ]] || { echo "ROS 2 Jazzy absent." >&2; exit 1; }
    set +u
    source /opt/ros/jazzy/setup.bash
    if [[ -n "${FOSA_BASE_WS:-}" ]]; then
        for script in "$FOSA_BASE_WS/install/setup.bash" "$FOSA_BASE_WS/setup.bash"; do
            if [[ -f "$script" ]]; then source "$script"; break; fi
        done
    fi
    set -u
    for paquet in "${PAQUETS_ROS[@]}"; do
        ros2 pkg prefix "$paquet" >/dev/null 2>&1 || manquants+=("$paquet")
    done
    if ((${#manquants[@]})); then
        echo "Paquets ROS absents : ${manquants[*]}" >&2
        echo "À demander aux organisateurs (installation système) :" >&2
        echo "  sudo apt install ros-jazzy-navigation2 ros-jazzy-slam-toolbox" >&2
        [[ "${FOSA_IGNORER_PREREQUIS:-0}" == "1" ]] || exit 1
    fi
    python3 -c "import cv2; cv2.aruco.DICT_APRILTAG_36h11" \
        || { echo "OpenCV avec aruco absent (python3-opencv)." >&2; exit 1; }
}

installer_vosk() {
    python3 -m venv --system-site-packages "$ESPACE_TRAVAIL/.venv"
    local pip=("$ESPACE_TRAVAIL/.venv/bin/python" -m pip install --quiet)
    if [[ -d "$LIVRAISON/roues" ]]; then
        "${pip[@]}" --no-index --find-links "$LIVRAISON/roues" vosk
    else
        "${pip[@]}" vosk
    fi
    mkdir -p "$DOSSIER_MODELES"
    if [[ ! -d "$DOSSIER_MODELES/$MODELE" ]]; then
        local archive="$LIVRAISON/$MODELE.zip"
        [[ -f "$archive" ]] || { mkdir -p "$LIVRAISON"; curl -fL "$URL_MODELE" -o "$archive"; }
        python3 -m zipfile -e "$archive" "$DOSSIER_MODELES"
    fi
}

construire() {
    cd "$ESPACE_TRAVAIL"
    set +u
    colcon build --symlink-install --base-paths "$DOSSIER_SCRIPT" \
        --packages-up-to caytu_course
    set -u
}

case "${1:-}" in
    --preparer) preparer ;;
    "")
        echo "[1/3] Prérequis (aucune installation système)..."
        verifier_prerequis
        echo "[2/3] Vosk et modèle du mot de départ..."
        installer_vosk
        echo "[3/3] Construction des paquets de la course..."
        construire
        cat <<FIN

Déploiement terminé. Pour chaque créneau (le robot redémarre entre deux) :
  cd $DOSSIER_SCRIPT
  ./run.sh diagnostic-robot          # premier créneau : vérifier les interfaces
  ./run.sh course                     # lancer la solution (attend ordre + départ)
  ./run.sh depart                     # départ par commande (sinon dire « allez »)
FIN
        ;;
    *) echo "Usage : $0 [--preparer]" >&2; exit 2 ;;
esac
