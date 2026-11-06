#!/usr/bin/env bash
# Déploiement sur le robot PARC (Jetson AGX Orin, ROS 2 Jazzy) SANS sudo.
#
# Règlement : tout dans le dossier de l'équipe, rien dans le système ; les
# paquets système (Nav2, slam_toolbox) se demandent aux organisateurs.
#
#   Sur le PC :                  ./deployer_robot.sh --envoyer ROBOT_IP
#       archive uniquement src/, copie fosa.zip vers sitoe@ROBOT_IP,
#       remplace ~/teams/fosa/src et compile dans ~/teams/fosa sur le robot.
#   Sur le PC, avec Internet :  ./deployer_robot.sh --preparer
#       télécharge dans caytu_config/config/livraison les paquets Python de Vosk
#       pour aarch64 / Python 3.12 et le modèle FR. Facultatif.
#   Sur le robot :               ./deployer_robot.sh
#       vérifie les prérequis, construit les paquets de la course, puis
#       installe Vosk dans <workspace>/.fosa_python (sans venv ni pip requis).
#       La voix est facultative : un échec n'empêche pas la course.
set -euo pipefail

DOSSIER_SCRIPT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ESPACE_TRAVAIL="$(cd -- "$DOSSIER_SCRIPT/../.." && pwd)"
EQUIPE="fosa"
UTILISATEUR_ROBOT="sitoe"
LIVRAISON="${FOSA_LIVRAISON:-$DOSSIER_SCRIPT/caytu_config/config/livraison}"
CIBLE_PYTHON="$ESPACE_TRAVAIL/.fosa_python"
MODELE="vosk-model-small-fr-0.22"
URL_MODELE="https://alphacephei.com/vosk/models/$MODELE.zip"
DOSSIER_MODELES="$DOSSIER_SCRIPT/caytu_config/config/models"
PAQUETS_ROS=(nav2_controller nav2_planner nav2_behaviors nav2_bt_navigator
    nav2_lifecycle_manager nav2_smac_planner nav2_regulated_pure_pursuit_controller
    nav2_costmap_2d nav2_msgs slam_toolbox cv_bridge tf2_ros)
PLATEFORME=(--platform manylinux2014_aarch64 --platform manylinux_2_17_aarch64
    --python-version 3.12 --implementation cp --only-binary=:all:)

afficher_aide() {
    cat <<AIDE
Usage : ./deployer_robot.sh [MODE]

Sur le PC :
  --envoyer ROBOT_IP   archive uniquement src/, copie vers sitoe@ROBOT_IP,
                       puis extrait et compile dans ~/teams/fosa sur le Jetson.
  --preparer           prépare les dépendances et le modèle Vosk facultatifs
                       dans caytu_config/config/livraison (à faire avant --envoyer).

Sur le robot, depuis ~/teams/fosa/src/fosa-solution :
  (sans argument)     vérifie les prérequis, compile et installe Vosk si prêt.

Équipe : fosa. Compte SSH : sitoe. Les paquets système manquants sont à
demander aux organisateurs ; ce script n'utilise pas sudo.
AIDE
}

preparer() {
    local roues="$LIVRAISON/roues"
    mkdir -p "$roues"
    echo "[1/3] Vosk (roue aarch64, sans dépendances)..."
    python3 -m pip download vosk --no-deps --dest "$roues" "${PLATEFORME[@]}"
    echo "[2/3] Dépendances de Vosk (roues aarch64 ; srt n'existe qu'en source)..."
    python3 -m pip download cffi requests tqdm websockets --dest "$roues" "${PLATEFORME[@]}"
    python3 -m pip download srt --no-deps --no-binary=:all: --dest "$roues"
    echo "[3/3] Modèle Vosk $MODELE..."
    [[ -f "$LIVRAISON/$MODELE.zip" ]] || curl -fsSL "$URL_MODELE" -o "$LIVRAISON/$MODELE.zip"
    echo "Puis : ./deployer_robot.sh --envoyer ROBOT_IP"
}

envoyer() {
    local ip="${1:-}" archive="$ESPACE_TRAVAIL/$EQUIPE.zip"
    local cible
    if [[ -z "$ip" || -n "${2:-}" ]]; then
        echo "Usage : $0 --envoyer ROBOT_IP" >&2
        exit 2
    fi
    if [[ ! -d "$ESPACE_TRAVAIL/src" ]]; then
        echo "Dossier src absent dans le workspace $ESPACE_TRAVAIL." >&2
        exit 1
    fi
    for commande in zip scp ssh; do
        command -v "$commande" >/dev/null 2>&1 || {
            echo "Commande requise absente : $commande" >&2
            exit 1
        }
    done

    cible="$UTILISATEUR_ROBOT@$ip"
    rm -f "$archive"
    echo "[1/3] Archive des seules sources : $archive"
    (cd "$ESPACE_TRAVAIL" && zip -qr "$archive" src) || exit 1
    echo "[2/3] Copie vers $cible:~/$(basename "$archive")"
    scp "$archive" "$cible:~/$(basename "$archive")" || exit 1
    echo "[3/3] Extraction et compilation dans ~/teams/$EQUIPE sur le robot..."
    ssh "$cible" 'bash -s' <<REMOTE
set -eo pipefail
[[ -f "\$HOME/ros2_ws/install/setup.bash" ]] || {
    echo "Workspace ROS absent : \$HOME/ros2_ws/install/setup.bash" >&2
    exit 1
}
command -v unzip >/dev/null 2>&1 || {
    echo "unzip est absent sur le robot ; demander son installation aux organisateurs." >&2
    exit 1
}
source "\$HOME/ros2_ws/install/setup.bash"
command -v colcon >/dev/null 2>&1 || {
    echo "colcon est absent après le chargement du workspace ROS." >&2
    exit 1
}
mkdir -p "\$HOME/teams/$EQUIPE"
cd "\$HOME/teams/$EQUIPE"
rm -rf src
unzip -q "\$HOME/$EQUIPE.zip"
source "\$HOME/ros2_ws/install/setup.bash"
colcon build
source install/setup.bash
echo "Sources compilées dans \$HOME/teams/$EQUIPE."
REMOTE
    cat <<FIN

Déploiement terminé. Archive conservée sur le PC : $archive
Étape suivante, sur le PC :
  ssh $cible
  cd ~/teams/$EQUIPE/src/fosa-solution
  ./run.sh diagnostic-robot
FIN
}

sourcer_ros() {
    set +u
    source /opt/ros/jazzy/setup.bash
    local base="${FOSA_BASE_WS:-$HOME/ros2_ws}"
    if [[ "$(cd "$base" 2>/dev/null && pwd)" != "$ESPACE_TRAVAIL" ]]; then
        for script in "$base/install/setup.bash" "$base/setup.bash"; do
            if [[ -f "$script" ]]; then source "$script"; break; fi
        done
    fi
    set -u
}

verifier_prerequis() {
    local manquants=()
    [[ -f /opt/ros/jazzy/setup.bash ]] || { echo "ROS 2 Jazzy absent." >&2; exit 1; }
    sourcer_ros
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

construire() {
    cd "$ESPACE_TRAVAIL"
    set +u
    colcon build
    source install/setup.bash
    set -u
}

# Sans pip : une roue est une archive zip déjà installable par extraction.
extraire_sans_pip() {
    local roue archive
    for roue in "$LIVRAISON/roues"/*.whl; do
        python3 -m zipfile -e "$roue" "$CIBLE_PYTHON"
    done
    for archive in "$LIVRAISON/roues"/srt-*.tar.gz; do
        tar -xzf "$archive" -C "$CIBLE_PYTHON" --wildcards --strip-components=1 '*/srt.py'
    done
}

installer_vosk() {
    mkdir -p "$CIBLE_PYTHON"
    if [[ -d "$LIVRAISON/roues" ]]; then
        if python3 -m pip --version >/dev/null 2>&1; then
            python3 -m pip install --quiet --no-index --no-build-isolation \
                --find-links "$LIVRAISON/roues" --target "$CIBLE_PYTHON" vosk
        else
            extraire_sans_pip
        fi
    else
        python3 -m pip install --quiet --target "$CIBLE_PYTHON" vosk
    fi
    PYTHONPATH="$CIBLE_PYTHON${PYTHONPATH:+:$PYTHONPATH}" python3 -c "import vosk"
    mkdir -p "$DOSSIER_MODELES"
    if [[ ! -d "$DOSSIER_MODELES/$MODELE" ]]; then
        local archive="$LIVRAISON/$MODELE.zip"
        [[ -f "$archive" ]] || { mkdir -p "$LIVRAISON"; curl -fsSL "$URL_MODELE" -o "$archive"; }
        python3 -m zipfile -e "$archive" "$DOSSIER_MODELES"
    fi
}

case "${1:-}" in
    --preparer) preparer ;;
    --envoyer) shift; envoyer "$@" ;;
    "")
        # Sur le PC, ce build (--symlink-install) se mélangerait à colcon build.
        if [[ "$(uname -m)" != aarch64 && "${FOSA_FORCER_PC:-0}" != 1 ]]; then
            echo "Ce script s'exécute SUR LE ROBOT. Sur le PC : colcon build, ou" >&2
            echo "  $0 --envoyer ROBOT_IP" >&2
            exit 1
        fi
        echo "[1/3] Prérequis (aucune installation système)..."
        verifier_prerequis
        echo "[2/3] Construction des paquets de la course..."
        construire
        echo "[3/3] Vosk et modèle du mot de départ (facultatif)..."
        if ! (installer_vosk); then
            echo "ATTENTION : voix indisponible, départ par commande uniquement." >&2
        fi
        cat <<FIN

Déploiement terminé. Pour chaque créneau (le robot redémarre entre deux) :
  cd $DOSSIER_SCRIPT
  ./run.sh diagnostic-robot          # premier créneau : vérifier les interfaces
  ./run.sh course                     # lancer la solution (attend ordre + départ)
  ./run.sh depart                     # départ par commande (sinon dire « allez »)
FIN
        ;;
    --help|-h|aide) afficher_aide ;;
    *) afficher_aide >&2; exit 2 ;;
esac
