#!/usr/bin/env bash
# Prépare un poste Ubuntu 24.04 (PC de l'équipe, pas le robot) pour construire
# et utiliser la solution Fosa de la phase finale. Le robot n'est jamais
# installé par ce script : voir deploy/ (deploy.sh, installer_robot.sh).
set -euo pipefail

DOSSIER_SCRIPT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ESPACE_TRAVAIL="$(cd -- "$DOSSIER_SCRIPT/../.." && pwd)"
DOSSIER_SOURCES="$ESPACE_TRAVAIL/src"
DEPOT_PARC="$DOSSIER_SOURCES/PARC2026-Engineers-League"
ROS_DISTRO="jazzy"
TELECHARGER_MODELES=""

afficher_aide() {
    cat <<'AIDE'
Usage : ./install.sh [--models | --no-models] [--help]

Installe ce que la course de la phase finale utilise : ROS 2 Jazzy, Nav2,
slam_toolbox, le garde-fou (nav2_collision_monitor), les AprilTags (OpenCV et
apriltag_ros), depth_image_proc, audio_common_msgs, le dépôt officiel PARC
(simulateur, arbitre parc_referee, messages parc_msgs) et faster-whisper pour
la reconnaissance du mot de départ. Compile le workspace.

Options :
  --models      télécharger dès maintenant le modèle Whisper du mot de départ
                (course.voice.model de caytu_config, « base »)
  --no-models   reporter ce téléchargement
  --help        afficher cette aide

Pour récupérer le modèle plus tard : ./install.sh --models
AIDE
}

while (($#)); do
    case "$1" in
        --models) TELECHARGER_MODELES="oui" ;;
        --no-models) TELECHARGER_MODELES="non" ;;
        --help|-h) afficher_aide; exit 0 ;;
        *) echo "Option inconnue : $1" >&2; afficher_aide >&2; exit 2 ;;
    esac
    shift
done

if [[ -z "$TELECHARGER_MODELES" && -t 0 ]]; then
    read -r -p "Télécharger maintenant le modèle Whisper du mot de départ ? [o/N] " reponse
    case "${reponse,,}" in o|oui|y|yes) TELECHARGER_MODELES="oui" ;; *) TELECHARGER_MODELES="non" ;; esac
else
    TELECHARGER_MODELES="${TELECHARGER_MODELES:-non}"
fi

if [[ ! -r /etc/os-release ]]; then
    echo "Impossible d'identifier le système. L'installation vise Ubuntu 24.04." >&2
    exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != "24.04" ]]; then
    echo "Ce script requiert Ubuntu 24.04 (détecté : ${PRETTY_NAME:-inconnu})." >&2
    echo "ROS 2 Jazzy est installé ici depuis les paquets Ubuntu officiels." >&2
    exit 1
fi

if [[ ! -d "$DOSSIER_SOURCES" ]]; then
    echo "Dossier de workspace absent : $DOSSIER_SOURCES" >&2
    echo "Placez ce dépôt dans <workspace>/src/fosa-solution avant l'installation." >&2
    exit 1
fi

if ! command -v sudo >/dev/null 2>&1; then
    echo "sudo est requis pour installer ROS 2 et les dépendances système." >&2
    exit 1
fi

echo "[1/6] Installation des outils Ubuntu nécessaires..."
sudo apt-get update
sudo apt-get install -y ca-certificates curl git wget build-essential \
    python3-pip python3-venv python3-yaml python3-rosdep \
    portaudio19-dev alsa-utils

if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    echo "[2/6] Installation du dépôt officiel et de ROS 2 Jazzy..."
    VERSION_SOURCE_ROS="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
    PAQUET_SOURCE_ROS="/tmp/ros2-apt-source_${VERSION_SOURCE_ROS}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
    curl -fsSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VERSION_SOURCE_ROS}/$(basename "$PAQUET_SOURCE_ROS")" \
        -o "$PAQUET_SOURCE_ROS"
    sudo dpkg -i "$PAQUET_SOURCE_ROS"
    sudo apt-get update
    sudo apt-get install -y "ros-${ROS_DISTRO}-desktop" ros-dev-tools
else
    echo "[2/6] ROS 2 Jazzy est déjà installé."
fi

echo "[3/6] Initialisation de rosdep..."
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update

mkdir -p "$DOSSIER_SOURCES"
# find rend toujours 0 : c'est la liste des fichiers trouvés qu'il faut tester.
if [[ -z "$(find "$DOSSIER_SOURCES" -name package.xml \
        -exec grep -l '<name>parc_robot_bringup</name>' {} + 2> /dev/null | head -n 1)" ]]; then
    if [[ ! -d "$DEPOT_PARC/.git" ]]; then
        echo "[4/6] Récupération du simulateur officiel PARC..."
        git clone https://github.com/PARC-Robotics/PARC2026-Engineers-League.git "$DEPOT_PARC"
    fi
else
    echo "[4/6] Simulateur officiel PARC déjà présent."
fi
# Mise à jour du 5 novembre : arbitre (parc_referee), ses messages (parc_msgs)
# et simulateur aux topics du vrai robot. Un ancien clone est mis à jour.
if [[ -d "$DEPOT_PARC/.git" ]]; then
    git -C "$DEPOT_PARC" pull --ff-only \
        || echo "Mise à jour du dépôt PARC impossible (modifications locales ?) : git -C $DEPOT_PARC pull" >&2
fi
if [[ -z "$(grep -rl --include=package.xml '<name>parc_msgs</name>' "$DOSSIER_SOURCES" 2> /dev/null | head -n 1)" ]]; then
    echo "ATTENTION : parc_msgs absent de $DOSSIER_SOURCES : l'arbitre officiel ne pourra pas" >&2
    echo "être essayé sur ce PC (git pull du dépôt PARC2026-Engineers-League)." >&2
fi

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u
# Les package.xml déclarent ce que la course utilise : Nav2, slam_toolbox,
# nav2_collision_monitor, laser_filters, depth_image_proc, cv_bridge,
# apriltag_ros, audio_common_msgs...
rosdep install --from-paths "$DOSSIER_SOURCES" --ignore-src --rosdistro "$ROS_DISTRO" -r -y

echo "[5/6] Reconnaissance vocale (faster-whisper) dans l'environnement Python du workspace..."
python3 -m venv --system-site-packages "$ESPACE_TRAVAIL/.venv"
# ROS fournit NumPy/OpenCV via apt ; l'environnement partage ces paquets système.
# sounddevice : micro du PC, pour essayer le mot de départ sans le ReSpeaker.
# brotli >= 1.2 : le téléchargement du modèle (huggingface_hub -> httpx2) passe
# output_buffer_limit à brotli ; le python3-brotli 1.1 d'Ubuntu 24.04, visible
# dans ce .venv, échoue (« process() takes no keyword arguments »).
"$ESPACE_TRAVAIL/.venv/bin/python" -m pip install --upgrade pip
"$ESPACE_TRAVAIL/.venv/bin/python" -m pip install faster-whisper sounddevice "brotli>=1.2"

echo "[6/6] Compilation de tous les paquets présents dans le workspace..."
(
    cd "$ESPACE_TRAVAIL"
    colcon build --symlink-install --base-paths "$DOSSIER_SOURCES"
)

# Détecteur des tags de la solution (apriltag_ros) et messages de l'arbitre.
set +u
source "$ESPACE_TRAVAIL/install/setup.bash"
set -u
for paquet in apriltag_ros apriltag_msgs parc_msgs parc_referee; do
    ros2 pkg prefix "$paquet" > /dev/null 2>&1 \
        || echo "ATTENTION : $paquet introuvable après l'installation (apriltag_ros : sudo apt install ros-$ROS_DISTRO-apriltag-ros)." >&2
done

if [[ "$TELECHARGER_MODELES" == "oui" ]]; then
    # Modèle du mot de départ, lu dans caytu_config/config/fosa.yaml.
    readarray -t MODELE_WHISPER < <("$ESPACE_TRAVAIL/.venv/bin/python" - "$DOSSIER_SCRIPT/caytu_config/config/fosa.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as fichier:
    voix = yaml.safe_load(fichier)["course"]["voice"]
print(voix["model"])
print(voix.get("device", "cpu"))
print(voix.get("compute_type", "int8"))
PY
)
    echo "Téléchargement du modèle Whisper du mot de départ : ${MODELE_WHISPER[0]}"
    "$ESPACE_TRAVAIL/.venv/bin/python" - "${MODELE_WHISPER[@]}" <<'PY'
import sys
from faster_whisper import WhisperModel

WhisperModel(sys.argv[1], device=sys.argv[2], compute_type=sys.argv[3])
PY
else
    echo "Téléchargement du modèle Whisper reporté. Lancez ./install.sh --models quand vous le souhaitez."
fi

cat <<FIN

Installation terminée.
Workspace : $ESPACE_TRAVAIL
Lancement : $DOSSIER_SCRIPT/run.sh
Vérifier toute la course sans le robot : $DOSSIER_SCRIPT/simulation/banc.sh 2d tout
Course jugée par l'arbitre officiel : $DOSSIER_SCRIPT/simulation/banc.sh 2d arbitre
Paquets pour un robot sans Internet : $DOSSIER_SCRIPT/deploy/telecharger_pour_robot.sh
Sans run.sh, dans chaque nouveau terminal :
  source /opt/ros/$ROS_DISTRO/setup.bash
  source $ESPACE_TRAVAIL/install/setup.bash
  source $ESPACE_TRAVAIL/.venv/bin/activate
  export PYTHONPATH=$ESPACE_TRAVAIL/.venv/lib/python3.12/site-packages:\$PYTHONPATH
(la dernière ligne permet aux nœuds lancés par ros2 launch, dont mot_depart,
de trouver faster-whisper ; run.sh le fait tout seul)
FIN
