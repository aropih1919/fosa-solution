#!/usr/bin/env bash
# Sur le PORTABLE, avec Internet : télécharge ce que la solution doit installer
# sur le robot, pour le cas où le robot n'a pas Internet.
#
#   deploy/telecharger_pour_robot.sh                 interroge le robot (SSH) puis télécharge
#   deploy/telecharger_pour_robot.sh --sans-robot    robot injoignable : Jetson par défaut
#        [--python 3.12] [--arch aarch64] [--glibc 2.39] [--numpy 1.26.4]
#   option : --aruco   ajoute OpenCV contrib (si l'OpenCV du robot n'a pas aruco)
#
# Résultat dans robot_data/paquets/ (non versionné) :
#   roues/      faster-whisper et ses dépendances, compilées pour le robot
#               (aarch64, Python du robot, numpy du robot inchangé)
#   hf/hub/     le modèle Whisper du mot de départ (course.voice.model)
# Puis, au créneau : deploy/deploy.sh --paquets --installer [--map <carte>]
# (envoi dans le dossier d'équipe du robot, installation sans Internet).
#
# Les paquets apt (ROS, colcon...) ne sont pas concernés : ils vont dans le
# système du robot et se demandent aux organisateurs (installer_robot.sh donne
# la commande).
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$DEPLOY_DIR")"
LOCAL_DATA="${FOSA_LOCAL_DATA:-$REPO_DIR/robot_data}"
SORTIE="$LOCAL_DATA/paquets"

SANS_ROBOT=0; ARUCO=0
PYTHON_ROBOT="3.12"; ARCH="aarch64"; GLIBC="2.39"; NUMPY="1.26.4"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --sans-robot) SANS_ROBOT=1; shift ;;
        --python) PYTHON_ROBOT="$2"; shift 2 ;;
        --arch) ARCH="$2"; shift 2 ;;
        --glibc) GLIBC="$2"; shift 2 ;;
        --numpy) NUMPY="$2"; shift 2 ;;
        --aruco) ARUCO=1; shift ;;
        -h|--help) sed -n '2,/^[^#]/{/^#/s/^# \{0,1\}//p}' "$0"; exit 0 ;;
        *) echo "Option inconnue : $1 (voir --help)" >&2; exit 1 ;;
    esac
done

# Python du portable qui télécharge : celui du workspace (install.sh) s'il existe.
PY=python3
for candidat in "$REPO_DIR/../../.venv/bin/python" "$REPO_DIR/.venv/bin/python"; do
    [[ -x "$candidat" ]] && { PY="$candidat"; break; }
done
"$PY" -m pip --version > /dev/null 2>&1 || { echo "pip absent de $PY (sudo apt install python3-pip)." >&2; exit 1; }

mkdir -p "$SORTIE"
CONTRAINTES="$SORTIE/contraintes_robot.txt"

# ------------------------------------------------------------ le robot
if (( ! SANS_ROBOT )); then
    # shellcheck disable=SC1091
    source "$DEPLOY_DIR/common.sh"
    echo "== Interrogation du robot $ROBOT"
    # Versions des paquets Python du SYSTÈME du robot (hors notre dossier) : les
    # roues téléchargées les respectent, rien ne masquera un module du système.
    # Python vu par nos nœuds : après l'espace de travail de base (comme env.sh).
    robot_ssh "source '$ROBOT_BASE_SETUP' > /dev/null 2>&1; python3 - '$ROBOT_DIR'" \
        > "$SORTIE/robot.txt" <<'PYTHON'
import os, platform, sys
import importlib.metadata as md
cible = os.path.realpath(sys.argv[1])
print('python', '%d.%d' % sys.version_info[:2])
print('arch', platform.machine())
print('glibc', platform.libc_ver()[1] or '2.17')
chemins = [p for p in sys.path if not os.path.realpath(p or '.').startswith(cible)]
vus = set()
for dist in md.distributions(path=chemins):
    nom, version = dist.metadata['Name'], dist.version
    if nom and version and nom.lower() not in vus:
        vus.add(nom.lower())
        print('paquet', nom, version)
PYTHON
    PYTHON_ROBOT=$(awk '$1=="python"{print $2}' "$SORTIE/robot.txt")
    ARCH=$(awk '$1=="arch"{print $2}' "$SORTIE/robot.txt")
    GLIBC=$(awk '$1=="glibc"{print $2}' "$SORTIE/robot.txt")
    awk '$1=="paquet"{print $2"=="$3}' "$SORTIE/robot.txt" | grep -viE '^brotli==' > "$CONTRAINTES" || true
    NUMPY=$(awk 'tolower($1)=="paquet" && tolower($2)=="numpy"{print $3}' "$SORTIE/robot.txt")
else
    echo "numpy==$NUMPY" > "$CONTRAINTES"
fi
echo "Robot : Python $PYTHON_ROBOT, $ARCH, glibc $GLIBC, numpy ${NUMPY:-absent}"

# Étiquettes de plateforme acceptées par le robot (manylinux jusqu'à sa glibc).
PLATEFORMES=(--platform "linux_$ARCH" --platform "manylinux2014_$ARCH")
mineure_max=${GLIBC#*.}; mineure_max=${mineure_max%%.*}
for ((m = 17; m <= mineure_max; m++)); do PLATEFORMES+=(--platform "manylinux_2_${m}_$ARCH"); done
ABI="cp${PYTHON_ROBOT/./}"
OPTIONS=("${PLATEFORMES[@]}" --python-version "$PYTHON_ROBOT" --implementation cp
         --abi "$ABI" --abi abi3 --abi none --only-binary=:all: -d "$SORTIE/roues")

PAQUETS=(faster-whisper "brotli>=1.2")
if (( ARUCO )); then
    # Même règle qu'installer_robot.sh : 4.12+ exige numpy 2.
    if [[ "${NUMPY:-1}" == 1.* ]]; then PAQUETS+=("opencv-contrib-python-headless==4.10.0.84")
    else PAQUETS+=(opencv-contrib-python-headless); fi
fi

echo "== Téléchargement de ${PAQUETS[*]} pour le robot"
rm -rf "$SORTIE/roues"; mkdir -p "$SORTIE/roues"
if ! "$PY" -m pip download -q "${OPTIONS[@]}" -c "$CONTRAINTES" "${PAQUETS[@]}"; then
    echo "Versions du système du robot introuvables en roues : nouvel essai en ne gardant que numpy."
    if [[ -n "$NUMPY" ]]; then echo "numpy==$NUMPY" > "$CONTRAINTES"; else : > "$CONTRAINTES"; fi
    "$PY" -m pip download -q "${OPTIONS[@]}" -c "$CONTRAINTES" "${PAQUETS[@]}"
fi
echo "$(ls "$SORTIE/roues" | wc -l) roues, $(du -sh "$SORTIE/roues" | cut -f1)"

# ------------------------------------------------------------ modèle Whisper
MODELE=$("$PY" - "$REPO_DIR/caytu_config/config/fosa.yaml" <<'PYTHON'
import sys, yaml
print(yaml.safe_load(open(sys.argv[1], encoding='utf-8'))['course']['voice']['model'])
PYTHON
)
echo "== Modèle Whisper « $MODELE » (indépendant de la machine)"
mkdir -p "$SORTIE/hf/hub"
if "$PY" -c "import faster_whisper" 2> /dev/null; then
    "$PY" - "$MODELE" "$SORTIE/hf/hub" <<'PYTHON'
import sys
from faster_whisper.utils import download_model
print(download_model(sys.argv[1], cache_dir=sys.argv[2]))
PYTHON
else
    echo "faster-whisper absent du portable : ./install.sh (ou pip install faster-whisper)," \
         "puis relancer ce script. Les roues sont prêtes, pas le modèle." >&2
    exit 1
fi
echo "$(du -sh "$SORTIE/hf" | cut -f1) de modèle"
echo
echo "Prêt dans $SORTIE. Au créneau : deploy/deploy.sh --paquets --installer [--map <carte>]"
