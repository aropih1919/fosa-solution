#!/usr/bin/env bash
# À lancer SUR LE ROBOT : vérifie ce qui manque à la solution Fosa et l'installe.
#
#   bash installer_robot.sh                 vérifie, puis installe après confirmation
#   bash installer_robot.sh --verifier      vérifie seulement, n'installe rien
#   bash installer_robot.sh --oui           installe sans poser de question
#   bash installer_robot.sh --sans-voix     ignore le mot de départ (faster-whisper)
#   bash installer_robot.sh --apriltag-ros  exige apriltag_ros même si caytu_config dit opencv
#
# Envoi depuis le portable (après deploy/deploy.sh, même s'il a échoué) :
#   ssh -p <port> <compte>@<robot> "cat > /dev/shm/fosa/installer_robot.sh" < deploy/installer_robot.sh
#   deploy/connect.sh
#   bash $FOSA_ROOT/installer_robot.sh
#
# Deux sortes d'installation :
#   - paquets apt (ROS, colcon, rsync, tmux) : il faut sudo. Ils vont dans le
#     système du robot et RESTENT après le redémarrage. Ce sont des paquets
#     publics, pas notre code, mais le robot est partagé : accord des
#     organisateurs d'abord ;
#   - faster-whisper, son modèle et, en dernier recours, OpenCV contrib : pip,
#     dans $FOSA_ROOT/py et $FOSA_ROOT/hf (notre dossier), effacés par
#     deploy/clean.sh. Il faut Internet sur le robot, SAUF si le portable les a
#     téléchargés (deploy/telecharger_pour_robot.sh) et envoyés
#     (deploy/deploy.sh --paquets) : ils sont alors pris dans $FOSA_ROOT/paquets.
#   - parc_msgs (messages de l'arbitre) ne s'installe pas ici : il est dans
#     ~/ros2_ws des organisateurs ; à défaut, deploy/deploy.sh --parc-msgs.
#
# Code de sortie : 0 tout est en place ; 1 il manque un élément indispensable ;
# 2 seul un élément optionnel manque.

set -o pipefail

VERIFIER_SEUL=0
OUI=0
VOIX=1
APRILTAG_ROS=0
APT_FAIT=0
for option in "$@"; do
    case "$option" in
        --verifier) VERIFIER_SEUL=1 ;;
        --oui) OUI=1 ;;
        --sans-voix) VOIX=0 ;;
        --apriltag-ros) APRILTAG_ROS=1 ;;
        -h|--help) sed -n '2,/^[^#]/{/^#/s/^# \{0,1\}//p}' "$0"; exit 0 ;;
        *) echo "Option inconnue : $option (voir --help)" >&2; exit 1 ;;
    esac
done

# --------------------------------------------------------------- environnement
# Celui de deploy.sh s'il existe (terminal de deploy/connect.sh), sinon ROS seul.
FOSA_ROOT="${FOSA_ROOT:-/dev/shm/fosa}"
# shellcheck disable=SC1090,SC1091
if [[ -z "${ROS_DISTRO:-}" ]]; then
    if [[ -f "$FOSA_ROOT/env.sh" ]]; then
        source "$FOSA_ROOT/env.sh" > /dev/null 2>&1
    else
        for setup in /opt/ros/jazzy/setup.bash /opt/ros/*/setup.bash; do
            [[ -f "$setup" ]] && { source "$setup"; break; }
        done
    fi
fi
if [[ -z "${ROS_DISTRO:-}" ]] || ! command -v ros2 > /dev/null; then
    echo "ROS 2 introuvable sur ce robot : à voir avec les organisateurs." >&2
    exit 1
fi
mkdir -p "$FOSA_ROOT/py" "$FOSA_ROOT/hf" && chmod 700 "$FOSA_ROOT"
case ":${PYTHONPATH:-}:" in
    *":$FOSA_ROOT/py:"*) ;;
    *) export PYTHONPATH="$FOSA_ROOT/py${PYTHONPATH:+:$PYTHONPATH}" ;;
esac
export HF_HOME="$FOSA_ROOT/hf"      # modèle Whisper : dans notre dossier, jamais dans ~/.cache
export PIP_NO_CACHE_DIR=1           # pas de cache pip dans le compte partagé
export PIP_DISABLE_PIP_VERSION_CHECK=1

# ------------------------------------------------------------ ce qu'il faut
ROS_INDISPENSABLES=(
    # construction de nos paquets
    ament_cmake ament_index_python rosidl_default_generators rosidl_default_runtime
    rosidl_runtime_py
    # bases ROS
    rclpy launch launch_ros ros2launch std_msgs std_srvs geometry_msgs sensor_msgs
    nav_msgs visualization_msgs action_msgs tf2_ros tf2_msgs rosbag2
    # carte, navigation, garde-fou
    slam_toolbox nav2_map_server nav2_amcl nav2_lifecycle_manager nav2_planner
    nav2_smac_planner nav2_controller nav2_regulated_pure_pursuit_controller
    nav2_dwb_controller nav2_mppi_controller nav2_behaviors nav2_bt_navigator
    nav2_behavior_tree nav2_costmap_2d nav2_msgs nav2_collision_monitor
    # capteurs
    laser_filters pointcloud_to_laserscan depth_image_proc cv_bridge
    # conduite au clavier pendant la cartographie
    teleop_twist_keyboard
)
ROS_VOIX=(audio_common_msgs)
ROS_APRILTAG=(apriltag_ros apriltag_msgs)

# Modèle Whisper : celui de caytu_config si la solution est compilée.
lire_modele_whisper() {
    python3 - 2> /dev/null <<'EOF' || echo "base cpu int8"
from caytu_config import load_config
voix = load_config().course.voice
print(voix.model, voix.device, voix.compute_type)
EOF
}
read -r MODELE DEVICE CALCUL < <(lire_modele_whisper)

# Détecteur des tags : celui de caytu_config (apriltag_ros par défaut).
lire_detecteur() {
    python3 -c "from caytu_config import load_config; print(load_config().perception.apriltag.detector)" \
        2> /dev/null || echo apriltag_ros
}
DETECTEUR=$(lire_detecteur)
[[ "$DETECTEUR" == apriltag_ros ]] && APRILTAG_ROS=1

manque() {   # niveau, nom affiché, paquet apt (vide : pas d'apt)
    if [[ "$1" == indispensable ]]; then MANQUE_INDISP+=("$2"); else MANQUE_OPTION+=("$2"); fi
    [[ -n "${3:-}" ]] && MANQUE_APT+=("$3")
    return 0
}

python_a() { python3 -c "$1" > /dev/null 2>&1; }

controler() {
    MANQUE_INDISP=(); MANQUE_OPTION=(); MANQUE_APT=()
    ARUCO_PIP=0; WHISPER_PIP=0; MODELE_A_CHARGER=0
    local liste paquet
    liste=$(ros2 pkg list 2> /dev/null)
    for paquet in "${ROS_INDISPENSABLES[@]}"; do
        grep -qx "$paquet" <<< "$liste" || manque indispensable "$paquet" "ros-$ROS_DISTRO-${paquet//_/-}"
    done
    if (( VOIX )); then
        for paquet in "${ROS_VOIX[@]}"; do
            grep -qx "$paquet" <<< "$liste" || manque optionnel "$paquet (voix)" "ros-$ROS_DISTRO-${paquet//_/-}"
        done
    fi
    if (( APRILTAG_ROS )); then
        # Détecteur des tags de la course : sans lui, aucune visite.
        for paquet in "${ROS_APRILTAG[@]}"; do
            grep -qx "$paquet" <<< "$liste" || manque indispensable "$paquet (détecteur des tags)" "ros-$ROS_DISTRO-${paquet//_/-}"
        done
    fi

    command -v colcon > /dev/null || manque indispensable "colcon (compilation)" python3-colcon-common-extensions
    command -v rsync > /dev/null || manque indispensable "rsync (deploy.sh)" rsync
    command -v tmux > /dev/null || manque optionnel "tmux (garde la course si SSH coupe)" tmux

    python_a "import yaml" || manque indispensable "python3 yaml" python3-yaml
    python_a "import numpy" || manque indispensable "python3 numpy" python3-numpy
    if ! python_a "import cv2"; then
        manque indispensable "python3 cv2 (OpenCV : image JPEG pour l'arbitre)" python3-opencv
    elif ! python_a "import cv2; cv2.aruco.getPredefinedDictionary"; then
        # aruco ne sert qu'au détecteur opencv, de secours quand apriltag_ros est choisi.
        if [[ "$DETECTEUR" == opencv ]]; then
            manque indispensable "cv2.aruco (détecteur opencv des tags)" ""
            ARUCO_PIP=1
        fi
    fi

    if (( VOIX )); then
        python_a "import pip" || manque optionnel "pip (pour faster-whisper)" python3-pip
        if ! python_a "import faster_whisper"; then
            manque optionnel "faster-whisper (voix)" ""
            WHISPER_PIP=1
            MODELE_A_CHARGER=1
        elif ! python_a "import brotli, sys; sys.exit(tuple(map(int, brotli.__version__.split('.')[:2])) < (1, 2))"; then
            manque optionnel "brotli >= 1.2 (téléchargement du modèle Whisper)" ""
            WHISPER_PIP=1
            MODELE_A_CHARGER=1
        elif ! compgen -G "$HF_HOME/hub/models--*faster-whisper-$MODELE*" > /dev/null; then
            manque optionnel "modèle Whisper « $MODELE » (voix)" ""
            MODELE_A_CHARGER=1
        fi
    fi
    # Arbitre officiel : /parc/start, /parc/tag_visit, /parc/finish, /parc/run_status.
    # (Sauf s'il vient d'être envoyé par deploy.sh --parc-msgs : compilé avec la solution.)
    if ! python_a "from parc_msgs.msg import TagOrder, RunStatus, StartSignal, TagVisit, Finish" \
            && [[ ! -f "$FOSA_ROOT/ws/src/parc_msgs/package.xml" ]]; then
        manque indispensable "parc_msgs (arbitre) : source ~/ros2_ws/install/setup.bash, sinon organisateurs ou deploy.sh --parc-msgs" ""
    fi
    SOLUTION_COMPILEE=0
    grep -qx caytu_nav_bringup <<< "$liste" && SOLUTION_COMPILEE=1
    return 0
}

afficher() {
    if (( ${#MANQUE_INDISP[@]} == 0 && ${#MANQUE_OPTION[@]} == 0 )); then
        echo "Tout est en place (ROS $ROS_DISTRO)."
    fi
    if (( ${#MANQUE_INDISP[@]} )); then
        echo "INDISPENSABLE absent :"
        printf '  - %s\n' "${MANQUE_INDISP[@]}"
    fi
    if (( ${#MANQUE_OPTION[@]} )); then
        echo "Optionnel absent :"
        printf '  - %s\n' "${MANQUE_OPTION[@]}"
    fi
    if (( ! SOLUTION_COMPILEE )) && [[ -z "${FOSA_DEPLOIEMENT:-}" ]]; then
        echo "Solution Fosa pas encore compilée ici : deploy/deploy.sh depuis le portable."
    fi
    return 0
}

code_sortie() {
    if (( ${#MANQUE_INDISP[@]} )); then return 1; fi
    if (( ${#MANQUE_OPTION[@]} )); then return 2; fi
    return 0
}

confirmer() {
    (( OUI )) && return 0
    if [[ ! -t 0 ]]; then
        echo "$1 -> non (pas de terminal : relancer avec --oui)."
        return 1
    fi
    local reponse
    read -r -p "$1 [o/N] " reponse
    [[ "$reponse" =~ ^[oOyY] ]]
}

# ------------------------------------------------------------ installations
installer_apt() {
    (( ${#MANQUE_APT[@]} )) || return 0
    local paquets=() dispo=() absents=() paquet sudo_cmd=""
    mapfile -t paquets < <(printf '%s\n' "${MANQUE_APT[@]}" | sort -u)
    echo
    echo "Paquets apt manquants : ${paquets[*]}"
    echo "Ils seraient installés dans le système du robot et y resteraient après le redémarrage."
    confirmer "Les installer avec apt ?" || {
        echo "Commande à transmettre aux organisateurs :"
        echo "  sudo apt-get update && sudo apt-get install -y ${paquets[*]}"
        return 1
    }
    if [[ $EUID -ne 0 ]]; then
        if ! command -v sudo > /dev/null || ! sudo -v; then
            echo "Pas de droits sudo sur ce compte. Commande à transmettre aux organisateurs :"
            echo "  sudo apt-get update && sudo apt-get install -y ${paquets[*]}"
            return 1
        fi
        sudo_cmd=sudo
    fi
    $sudo_cmd apt-get update -qq || echo "apt-get update a échoué (Internet ? dépôts ?) : on essaie quand même."
    for paquet in "${paquets[@]}"; do
        if apt-cache show "$paquet" > /dev/null 2>&1; then dispo+=("$paquet"); else absents+=("$paquet"); fi
    done
    if (( ${#absents[@]} )); then
        echo "Absents des dépôts apt du robot : ${absents[*]}"
        echo "  (paquets ros-$ROS_DISTRO-* : le dépôt packages.ros.org est-il configuré ?)"
    fi
    (( ${#dispo[@]} )) || return 1
    $sudo_cmd env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${dispo[@]}" \
        && APT_FAIT=1
}

version_numpy() { python3 -c "import numpy; print(numpy.__version__)" 2> /dev/null; }

# Paquets Python du système (hors $FOSA_ROOT/py) : « contraintes » écrit leurs
# versions pour pip ; « doublons » retire de $FOSA_ROOT/py les copies de même
# version, pour que rien n'y masque un module du système (cv_bridge, launch...).
paquets_systeme() {   # mode (contraintes | doublons), fichier
    python3 - "$1" "$FOSA_ROOT/py" "${2:-}" <<'PYTHON'
import os, sys
import importlib.metadata as md
mode, cible, fichier = sys.argv[1], os.path.realpath(sys.argv[2]), sys.argv[3]
systeme = [p for p in sys.path if not os.path.realpath(p or '.').startswith(cible)]
try:
    from pip._vendor.packaging.version import InvalidVersion, Version
except ImportError:
    Version = None
versions = {}
for dist in md.distributions(path=systeme):
    nom, version = dist.metadata['Name'], dist.version
    if not nom or not version:
        continue
    if Version is not None:
        try:
            Version(version)
        except InvalidVersion:
            continue
    versions.setdefault(nom.lower().replace('_', '-'), (nom, version))
if mode == 'contraintes':
    with open(fichier, 'w') as sortie:
        sortie.writelines(f'{nom}=={version}\n' for nom, version in sorted(versions.values()))
    sys.exit(0)
retires = []
for dist in list(md.distributions(path=[cible])):
    nom_affiche = dist.metadata['Name'] or ''
    nom = nom_affiche.lower().replace('_', '-')
    if not nom or versions.get(nom, (None, None))[1] != dist.version:
        continue
    dossier_info = os.path.realpath(str(dist._path))
    for fichier_dist in dist.files or []:
        chemin = os.path.realpath(os.path.join(cible, str(fichier_dist)))
        if chemin.startswith(cible + os.sep) and os.path.isfile(chemin):
            os.remove(chemin)
    if dossier_info.startswith(cible + os.sep) and os.path.isdir(dossier_info):
        for racine, _, fichiers in os.walk(dossier_info, topdown=False):
            for f in fichiers:
                os.remove(os.path.join(racine, f))
            os.rmdir(racine)
    retires.append(nom_affiche)
for racine, dossiers, fichiers in os.walk(cible, topdown=False):
    if racine != cible and not os.listdir(racine):
        os.rmdir(racine)
if retires:
    print('Gardés du système (copie retirée) : ' + ', '.join(sorted(retires)))
PYTHON
}

# Paquets téléchargés par le portable (deploy/telecharger_pour_robot.sh) : pip
# les prend là, sans Internet.
ROUES="$FOSA_ROOT/paquets/roues"
source_pip() {
    if compgen -G "$ROUES/*.whl" > /dev/null; then
        echo "--no-index --find-links $ROUES"
    fi
}

installer_pip() {   # paquets pip..., dans $FOSA_ROOT/py, sans masquer le système
    local contraintes="$FOSA_ROOT/contraintes_pip.txt" resultat numpy_sys source
    source=$(source_pip)
    [[ -n "$source" ]] && echo "Paquets pris dans $ROUES (sans Internet)."
    paquets_systeme contraintes "$contraintes" || : > "$contraintes"
    # brotli : celui d'Ubuntu 24.04 (1.1) fait échouer le téléchargement du
    # modèle ; on en installe un récent ici, sans le contraindre.
    sed -i '/^[Bb]rotli==/d' "$contraintes"
    # shellcheck disable=SC2086
    python3 -m pip install $source --target "$FOSA_ROOT/py" --upgrade -c "$contraintes" "$@"
    resultat=$?
    if (( resultat != 0 )); then
        echo "Versions du système incompatibles ou pip en échec : nouvel essai en ne gardant que numpy."
        numpy_sys=$(version_numpy)
        if [[ -n "$numpy_sys" ]]; then echo "numpy==$numpy_sys" > "$contraintes"; else : > "$contraintes"; fi
        # shellcheck disable=SC2086
        python3 -m pip install $source --target "$FOSA_ROOT/py" --upgrade -c "$contraintes" "$@"
        resultat=$?
    fi
    rm -f "$contraintes"
    paquets_systeme doublons
    if (( resultat != 0 )); then
        echo "pip a échoué (Internet sur le robot ?)."
    fi
    return $resultat
}

installer_aruco() {
    (( ARUCO_PIP )) || return 0
    echo
    echo "OpenCV du robot sans aruco : le détecteur opencv des tags ne peut pas tourner."
    echo "Autre solution : detecteur:=apriltag_ros (le défaut de caytu_config)."
    confirmer "Installer OpenCV contrib dans $FOSA_ROOT/py (dossier d'équipe, numpy inchangé) ?" || return 1
    local paquet=opencv-contrib-python-headless
    # Les versions 4.12 et suivantes exigent numpy 2 ; Ubuntu 24.04 a numpy 1.
    [[ "$(version_numpy)" == 1.* ]] && paquet="opencv-contrib-python-headless==4.10.0.84"
    # shellcheck disable=SC2046
    python3 -m pip install $(source_pip) --target "$FOSA_ROOT/py" --upgrade --no-deps "$paquet" \
        || echo "pip a échoué (Internet sur le robot ? sinon deploy/telecharger_pour_robot.sh --aruco)."
}

installer_whisper() {
    (( VOIX && WHISPER_PIP )) || return 0
    echo
    if ! python_a "import pip"; then
        echo "pip absent : faster-whisper non installé."
        return 1
    fi
    confirmer "Installer faster-whisper dans $FOSA_ROOT/py (dossier d'équipe, effacé par clean.sh) ?" \
        || return 1
    installer_pip faster-whisper "brotli>=1.2"
}

charger_modele() {
    (( VOIX && MODELE_A_CHARGER )) || return 0
    python_a "import faster_whisper" || return 1
    echo
    echo "Modèle Whisper « $MODELE » ($DEVICE, $CALCUL) : téléchargement dans $HF_HOME…"
    local erreurs="$FOSA_ROOT/erreur_whisper.txt"
    if python3 -c "from faster_whisper import WhisperModel; WhisperModel('$MODELE', device='$DEVICE', compute_type='$CALCUL')" 2> "$erreurs"; then
        echo "Modèle prêt : la course ne téléchargera rien."
        rm -f "$erreurs"
    else
        echo "Modèle non chargé (Internet sur le robot ? sinon deploy/telecharger_pour_robot.sh" \
             "puis deploy/deploy.sh --paquets) : $(tail -n 1 "$erreurs")"
        rm -f "$erreurs"
        return 1
    fi
}

# ------------------------------------------------------------------ déroulé
echo "== Vérification (ROS $ROS_DISTRO, espace $FOSA_ROOT)"
controler
afficher
if (( VERIFIER_SEUL )); then
    code_sortie
    exit $?
fi
if (( ${#MANQUE_INDISP[@]} == 0 && ${#MANQUE_OPTION[@]} == 0 )); then
    exit 0
fi

installer_apt
installer_aruco
installer_whisper
charger_modele

echo
echo "== Contrôle final"
controler
afficher
if (( APT_FAIT )); then
    echo "Paquets ajoutés au système : relancer deploy/deploy.sh (--map <carte>) depuis le"
    echo "portable si la compilation avait échoué ou si des paquets manquaient."
fi
code_sortie
exit $?
