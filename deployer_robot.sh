#!/usr/bin/env bash
# Livraison Fosa dans un workspace neuf ; activation seulement après compilation.
set -euo pipefail
DEPOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ESPACE_TRAVAIL="${FOSA_WS:-$(cd -- "$DEPOT/../.." && pwd)}"
source "$DEPOT/outils/config_exploitation.sh"
EQUIPE="$(lire_exploitation equipe)"
UTILISATEUR="$(lire_exploitation utilisateur_robot)"
DOSSIER_ROBOT="$(lire_exploitation dossier_robot)"
ROS_DISTRO="$(lire_exploitation ros_distro)"
LIVRAISON="$DEPOT/caytu_config/config/livraison"
MODELE="$(lire_exploitation modele_vosk)"
URL_MODELE="$(lire_exploitation url_modele_vosk)"

aide() {
    cat <<'AIDE'
Usage : ./deployer_robot.sh MODE
  --archive [fichier.zip]  sources Fosa seules + manifeste SHA-256 (sans robot)
  --envoyer ROBOT_IP      transfère, compile, active actuel (prépare la voix si absente)
  --preparer              télécharge Vosk ARM/Python et son modèle depuis le PC
  --installer            sur le Jetson : vérifie les dépendances, compile, installe la voix préparée
  --help                 affiche cette aide

FOSA_SANS_VOIX=1 avec --envoyer déploie sans préparer la voix (départ par commande).

Sur le robot : ~/teams/fosa/actuel/src/fosa-solution/run.sh
Les anciennes livraisons restent dans ~/teams/fosa/livraisons ; precedent pointe sur l'ancienne.
AIDE
}

preparer_voix() {
    local roues="$LIVRAISON/roues"
    mkdir -p "$roues"
    local paquets=()
    mapfile -t paquets < <(lire_exploitation paquets_python_voix)
    python3 -m pip download "${paquets[@]}" --no-deps --dest "$roues" \
        --platform manylinux2014_aarch64 --platform manylinux_2_17_aarch64 \
        --python-version "$(lire_exploitation python_robot)" --implementation cp --only-binary=:all:
    # Les dépendances pures et cffi sont résolues pour la cible, jamais pour le PC.
    python3 -m pip download cffi requests tqdm websockets --dest "$roues" \
        --platform manylinux2014_aarch64 --platform manylinux_2_17_aarch64 \
        --python-version "$(lire_exploitation python_robot)" --implementation cp --only-binary=:all:
    python3 -m pip download srt --no-deps --no-binary=:all: --dest "$roues"
    [[ -f "$LIVRAISON/$MODELE.zip" ]] || curl -fL "$URL_MODELE" -o "$LIVRAISON/$MODELE.zip"
    echo "Voix prête à livrer : $LIVRAISON"
}

archiver() {
    python3 "$DEPOT/outils/livraison.py" creer "$1"
    sha256sum "$1"
}

installer() {
    # Arrêter une éventuelle course précédente sans bloquer le déploiement.
    if [[ -x "$DEPOT/run.sh" ]]; then
        "$DEPOT/run.sh" arreter || echo "Arrêt précédent incomplet : compilation poursuivie." >&2
    fi
    if pgrep -af '[c]aytu_course/course_parc|[c]ourse\.launch\.py' >/dev/null; then
        echo "Des nœuds Fosa tournent encore : arrêtez-les puis relancez --installer." >&2
        return 1
    fi
    if [[ "$(uname -m)" != aarch64 && "${FOSA_FORCER_PC:-0}" != 1 ]]; then
        echo "--installer se lance sur le Jetson ; PC : --envoyer ROBOT_IP." >&2
        return 1
    fi
    set +u
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    base_ws="${FOSA_BASE_WS:-$HOME/parc_ws}"
    [[ -f "$base_ws/install/setup.bash" ]] || base_ws="$HOME/ros2_ws"
    if [[ ! -f "$base_ws/install/setup.bash" ]]; then
        echo "Workspace organisateur absent : source ~/parc_ws/install/setup.bash" >&2
        return 1
    fi
    source "$base_ws/install/setup.bash"
    set -u
    local paquet manquants=() paquets=()
    while read -r paquet; do
        ros2 pkg prefix "$paquet" >/dev/null 2>&1 || manquants+=("$paquet")
    done < <(lire_exploitation paquets_ros)
    if ((${#manquants[@]})); then
        echo "Paquets manquants à demander aux organisateurs : ${manquants[*]}" >&2
        return 1
    fi
    if ! python3 - <<'PYTHON'
import cv2
import numpy
import yaml
assert hasattr(cv2.aruco, "DICT_APRILTAG_36h11")
PYTHON
    then
        echo "Dépendances Python manquantes : installer python3-opencv python3-yaml python3-numpy." >&2
        return 1
    fi
    mapfile -t paquets < <(lire_exploitation paquets_fosa)
    (cd "$ESPACE_TRAVAIL" && colcon build --base-paths "$DEPOT" \
        --packages-select "${paquets[@]}" --executor sequential)
    if [[ ! -f "$ESPACE_TRAVAIL/install/setup.bash" ]]; then
        echo "Compilation incomplète : $ESPACE_TRAVAIL/install/setup.bash absent." >&2
        return 1
    fi
    if [[ -d "$LIVRAISON/roues" && -f "$LIVRAISON/$MODELE.zip" ]]; then
        # Installation hors ligne, sans venv et sans téléchargement sur le robot.
        mkdir -p "$ESPACE_TRAVAIL/.fosa_python" "$DEPOT/caytu_config/config/models"
        local fichier
        for fichier in "$LIVRAISON/roues/"*.whl; do
            python3 -m zipfile -e "$fichier" "$ESPACE_TRAVAIL/.fosa_python"
        done
        for fichier in "$LIVRAISON/roues/"srt-*.tar.gz; do
            tar -xzf "$fichier" -C "$ESPACE_TRAVAIL/.fosa_python" --wildcards --strip-components=1 '*/srt.py'
        done
        python3 -m zipfile -e "$LIVRAISON/$MODELE.zip" "$DEPOT/caytu_config/config/models"
        PYTHONPATH="$ESPACE_TRAVAIL/.fosa_python${PYTHONPATH:+:$PYTHONPATH}" \
            python3 -c 'import vosk; print("Vosk hors ligne installé.")'
    else
        echo "Voix non préparée ; départ par commande disponible."
    fi
}

envoyer() {
    local hote="${1:-}" archive="$ESPACE_TRAVAIL/$EQUIPE.zip" empreinte
    [[ "$hote" =~ ^[a-zA-Z0-9][a-zA-Z0-9.:-]*$ && $# == 1 ]] || {
        echo "Usage : --envoyer ROBOT_IP" >&2; return 2;
    }
    # La voix manquante est préparée sur le PC avant l'archive. Un déploiement
    # sans voix reste possible avec FOSA_SANS_VOIX=1 (départ par commande).
    if [[ "${FOSA_SANS_VOIX:-0}" != 1 ]] \
            && { [[ ! -f "$LIVRAISON/$MODELE.zip" ]] \
                 || ! compgen -G "$LIVRAISON/roues/*.whl" >/dev/null; }; then
        echo "Voix incomplète : préparation automatique (Vosk ARM + modèle)..."
        if ! preparer_voix; then
            echo "Préparation de la voix impossible ; déploiement sans voix." >&2
            echo "Relancez ./deployer_robot.sh --preparer, ou utilisez FOSA_SANS_VOIX=1 pour ignorer." >&2
        fi
    fi
    archiver "$archive"
    empreinte="$(sha256sum "$archive" | cut -d ' ' -f 1)"
    ssh "$UTILISATEUR@$hote" "mkdir -p \"\$HOME/$DOSSIER_ROBOT/entree\""
    scp "$archive" "$UTILISATEUR@$hote:$DOSSIER_ROBOT/entree/$empreinte.zip"
    ssh "$UTILISATEUR@$hote" bash -s -- "$DOSSIER_ROBOT" "$empreinte" <<'ROBOT'
set -euo pipefail
racine="$HOME/$1"
archive="$racine/entree/$2.zip"
[[ "$(sha256sum "$archive" | cut -d ' ' -f 1)" == "$2" ]]
mkdir -p "$racine/livraisons" "$racine/donnees"
livraison="$(mktemp -d "$racine/livraisons/${2:0:12}.XXXXXXXX")"
python3 - "$archive" "$livraison" <<'PYTHON'
import pathlib
import sys
import zipfile
with zipfile.ZipFile(sys.argv[1]) as archive:
    for info in archive.infolist():
        chemin = pathlib.PurePosixPath(info.filename)
        assert not chemin.is_absolute() and '..' not in chemin.parts
        assert info.filename == 'MANIFESTE.json' or info.filename.startswith('src/fosa-solution/')
    archive.extractall(sys.argv[2])
    for info in archive.infolist():
        pathlib.Path(sys.argv[2], info.filename).chmod((info.external_attr >> 16) & 0o777)
PYTHON
python3 "$livraison/src/fosa-solution/outils/livraison.py" verifier "$livraison"
bash "$livraison/src/fosa-solution/deployer_robot.sh" --installer
if [[ -L "$racine/actuel" ]]; then
    ln -sfn "$(readlink "$racine/actuel")" "$racine/precedent"
fi
ln -s "$livraison" "$racine/.actuel-$2"
mv -Tf "$racine/.actuel-$2" "$racine/actuel"
echo "Livraison active : $livraison"
ROBOT
    echo "Déploiement terminé : ssh $UTILISATEUR@$hote"
    echo "Puis : cd ~/$DOSSIER_ROBOT/actuel/src/fosa-solution && ./run.sh course"
}

case "${1:---help}" in
    --archive) archiver "${2:-$ESPACE_TRAVAIL/$EQUIPE.zip}" ;;
    --envoyer) shift; envoyer "$@" ;;
    --preparer) preparer_voix ;;
    --installer) installer ;;
    --help|-h) aide ;;
    *) aide >&2; exit 2 ;;
esac
