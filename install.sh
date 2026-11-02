#!/usr/bin/env bash
# Prépare un poste Ubuntu propre pour construire et utiliser la solution Fosa.
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

Installe ROS 2, les dépendances système et Python, le simulateur PARC,
construit les paquets du workspace et propose le téléchargement des modèles IA.

Options :
  --models      télécharger dès maintenant les modèles IA locaux configurés
  --no-models   reporter le téléchargement des modèles IA
  --help        afficher cette aide

Pour récupérer les modèles plus tard : ./install.sh --models
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
    read -r -p "Télécharger maintenant les modèles IA locaux (LLM, Whisper et voix Piper) ? [o/N] " reponse
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

echo "[1/7] Installation des outils Ubuntu nécessaires..."
sudo apt-get update
sudo apt-get install -y ca-certificates curl git wget build-essential \
    python3-pip python3-venv python3-yaml python3-rosdep \
    portaudio19-dev alsa-utils

if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    echo "[2/7] Installation du dépôt officiel et de ROS 2 Jazzy..."
    VERSION_SOURCE_ROS="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
    PAQUET_SOURCE_ROS="/tmp/ros2-apt-source_${VERSION_SOURCE_ROS}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
    curl -fsSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VERSION_SOURCE_ROS}/$(basename "$PAQUET_SOURCE_ROS")" \
        -o "$PAQUET_SOURCE_ROS"
    sudo dpkg -i "$PAQUET_SOURCE_ROS"
    sudo apt-get update
    sudo apt-get install -y "ros-${ROS_DISTRO}-desktop" ros-dev-tools
else
    echo "[2/7] ROS 2 Jazzy est déjà installé."
fi

echo "[3/7] Initialisation de rosdep et installation des dépendances ROS..."
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update

mkdir -p "$DOSSIER_SOURCES"
if ! find "$DOSSIER_SOURCES" -name package.xml \
    -exec grep -q -l '<name>parc_robot_bringup</name>' {} \; -quit; then
    if [[ ! -d "$DEPOT_PARC/.git" ]]; then
        echo "[4/7] Récupération du simulateur officiel PARC..."
        git clone https://github.com/PARC-Robotics/PARC2026-Engineers-League.git "$DEPOT_PARC"
    fi
fi

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u
rosdep install --from-paths "$DOSSIER_SOURCES" --ignore-src --rosdistro "$ROS_DISTRO" -r -y

echo "[5/7] Installation des bibliothèques audio et IA dans l'environnement Python du workspace..."
python3 -m venv --system-site-packages "$ESPACE_TRAVAIL/.venv"
# ROS fournit NumPy/OpenCV via apt; l'environnement partage ces paquets système.
"$ESPACE_TRAVAIL/.venv/bin/python" -m pip install --upgrade pip
"$ESPACE_TRAVAIL/.venv/bin/python" -m pip install faster-whisper piper-tts sounddevice

if ! command -v ollama >/dev/null 2>&1; then
    echo "[6/7] Installation d'Ollama (le modèle reste optionnel)..."
    curl -fsSL https://ollama.com/install.sh | sh
else
    echo "[6/7] Ollama est déjà installé."
fi

echo "[7/7] Compilation de tous les paquets présents dans le workspace..."
(
    cd "$ESPACE_TRAVAIL"
    colcon build --symlink-install --base-paths "$DOSSIER_SOURCES"
)

if [[ "$TELECHARGER_MODELES" == "oui" ]]; then
    set +u
    source "$ESPACE_TRAVAIL/install/setup.bash"
    set -u
    DOSSIER_CONFIG_INSTALLEE="$(ros2 pkg prefix caytu_config)/share/caytu_config/config"
    DOSSIER_MODELES="$DOSSIER_CONFIG_INSTALLEE/models"
    mkdir -p "$DOSSIER_MODELES"

    readarray -t MODELES_IA < <("$ESPACE_TRAVAIL/.venv/bin/python" - "$DOSSIER_SCRIPT/caytu_config/config/fosa.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as fichier:
    configuration = yaml.safe_load(fichier)
print(configuration["dialogue"]["llm"]["local_model"])
print(configuration["dialogue"]["stt"]["local_model"])
print(configuration["dialogue"]["tts"]["local_voice"])
PY
)

    echo "Téléchargement du modèle LLM configuré : ${MODELES_IA[0]}"
    ollama pull "${MODELES_IA[0]}"

    echo "Téléchargement du modèle Whisper configuré : ${MODELES_IA[1]}"
    "$ESPACE_TRAVAIL/.venv/bin/python" - "${MODELES_IA[1]}" <<'PY'
import sys
from faster_whisper import WhisperModel

WhisperModel(sys.argv[1], device="cpu", compute_type="int8")
PY

    NOM_VOIX="$(basename "${MODELES_IA[2]}")"
    RACINE_VOIX="${NOM_VOIX%.onnx}"
    IFS='-' read -r LANGUE VOIX QUALITE <<< "$RACINE_VOIX"
    CODE_LANGUE="${LANGUE%%_*}"
    CHEMIN_VOIX="${CODE_LANGUE,,}/${LANGUE}/${VOIX}/${QUALITE}"
    URL_VOIX="https://huggingface.co/rhasspy/piper-voices/resolve/main/$CHEMIN_VOIX/$NOM_VOIX"
    echo "Téléchargement de la voix Piper configurée : $NOM_VOIX"
    curl -fL "$URL_VOIX" -o "$DOSSIER_MODELES/$NOM_VOIX"
    curl -fL "${URL_VOIX}.json" -o "$DOSSIER_MODELES/$NOM_VOIX.json"
else
    echo "Téléchargement des modèles IA reporté. Lancez ./install.sh --models quand vous le souhaitez."
fi

cat <<FIN

Installation terminée.
Workspace : $ESPACE_TRAVAIL
Lancement : $DOSSIER_SCRIPT/run.sh
Les nouvelles fenêtres de terminal peuvent aussi sourcer :
  source /opt/ros/$ROS_DISTRO/setup.bash
  source $ESPACE_TRAVAIL/install/setup.bash
  source $ESPACE_TRAVAIL/.venv/bin/activate
FIN
