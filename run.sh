#!/usr/bin/env bash
# Lance un mode existant de la solution Fosa depuis un terminal préparé.
set -euo pipefail

DOSSIER_SCRIPT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ESPACE_TRAVAIL="$(cd -- "$DOSSIER_SCRIPT/../.." && pwd)"
ROS_DISTRO="jazzy"

afficher_aide() {
    cat <<'AIDE'
Usage : ./run.sh [MODE] [arguments ROS]

Modes de la course (phase réelle, robot réel par défaut) :
  course        SLAM + Nav2 + détection des tags + mot de départ + mission,
                détachée de la session SSH (tmux ou nohup), journal conservé
  arreter       arrêter la course lancée par « course » (essais)
  cartographie  idem sans la mission (essais : carte, réglages, rosbag)
  depart        donner le départ par commande (la mission doit attendre)
  ordre-test O  publier un ordre de test, ex. "7,1,9,4" (ESSAIS SEULEMENT)
  etat          suivre l'état de la course
  sauver-carte  sérialiser la carte SLAM (option : nom du fichier)
  diagnostic-robot  vérifier topics, TF, caméras, audio (premier créneau)

Profils : FOSA_PROFIL=simulation pour Gazebo (sinon robot réel).
Espace de travail de base du robot : FOSA_BASE_WS=<dossier> (sourcé d'abord).

Modes hérités (simulation, phase 1 et assistant) :
  complet       Gazebo PARC et assistant vocal complet
  navigation    tâche officielle surveillée, avec résultats conservés
  assistant     assistant complet sur un robot ou simulateur déjà actif
  tts [TEXTE]   prononcer un texte avec le fournisseur TTS configuré
  gazebo        terminal 1 : lancer task.launch.py, en affichant les erreurs
                terminal 2 : ./run.sh navigation
  benchmark     ouvrir l'outil de benchmark sur un monde déjà lancé
  compare       comparer les rapports présents dans results/
  diagnostic    lancer tools/diag_nav.py

Exemples :
  ./run.sh course
  ./run.sh depart
  FOSA_PROFIL=simulation ./run.sh course
  ./run.sh complet input_mode:=microphone
  ./run.sh assistant input_mode:=text
  ./run.sh gazebo
  ./run.sh navigation
  ./run.sh tts "Bonjour, ceci est un essai de synthèse vocale."

Options ROS supplémentaires (input_mode:=..., -p ..., etc.) après le mode
sont transmises au launch ou à l'exécutable concerné.
AIDE
}

mode="${1:-}"
if [[ -z "$mode" ]]; then
    if [[ -t 0 ]]; then
        echo "Que souhaitez-vous lancer ?"
        select choix in "course" "cartographie (sans mission)" "départ par commande" "diagnostic robot" "quitter"; do
            case "$REPLY" in
                1) mode="course"; break ;;
                2) mode="cartographie"; break ;;
                3) mode="depart"; break ;;
                4) mode="diagnostic-robot"; break ;;
                5) exit 0 ;;
                *) echo "Choisissez un numéro de 1 à 5." ;;
            esac
        done
    else
        afficher_aide >&2
        exit 2
    fi
fi
if (($#)); then shift; fi

if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    echo "ROS 2 Jazzy est absent. Lancez ./install.sh depuis ce dépôt." >&2
    exit 1
fi
if [[ ! -f "$ESPACE_TRAVAIL/install/setup.bash" ]]; then
    echo "Workspace non compilé : $ESPACE_TRAVAIL/install/setup.bash est absent." >&2
    echo "Lancez ./install.sh depuis ce dépôt." >&2
    exit 1
fi

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
# Espace de travail de base du robot (paquets des organisateurs) : sourcé avant
# le nôtre pour que nos paquets restent prioritaires.
if [[ -n "${FOSA_BASE_WS:-}" ]]; then
    for script in "$FOSA_BASE_WS/install/setup.bash" "$FOSA_BASE_WS/setup.bash"; do
        if [[ -f "$script" ]]; then source "$script"; break; fi
    done
fi
source "$ESPACE_TRAVAIL/install/setup.bash"
export FOSA_CONFIG_DIR="${FOSA_CONFIG_DIR:-$DOSSIER_SCRIPT/caytu_config/config}"
case "${FOSA_PROFIL:-robot}" in
    robot) ;;
    simulation)
        export FOSA_CONFIG_OVERRIDE="${FOSA_CONFIG_OVERRIDE:-$FOSA_CONFIG_DIR/profils/simulation.yaml}" ;;
    *) echo "Profil inconnu : $FOSA_PROFIL (robot | simulation)" >&2; exit 2 ;;
esac
if [[ -f "$ESPACE_TRAVAIL/.venv/bin/activate" ]]; then
    source "$ESPACE_TRAVAIL/.venv/bin/activate"
    DOSSIER_PAQUETS_PYTHON="$("$ESPACE_TRAVAIL/.venv/bin/python" -c \
        'import sys; print(f"python{sys.version_info.major}.{sys.version_info.minor}")')"
    DOSSIER_PAQUETS_PYTHON="$ESPACE_TRAVAIL/.venv/lib/$DOSSIER_PAQUETS_PYTHON/site-packages"
    if [[ -d "$DOSSIER_PAQUETS_PYTHON" ]]; then
        export PYTHONPATH="$DOSSIER_PAQUETS_PYTHON${PYTHONPATH:+:$PYTHONPATH}"
    fi
fi
if [[ -d "$ESPACE_TRAVAIL/.fosa_python" ]]; then
    # Vosk installé par deployer_robot.sh, sans venv.
    export PYTHONPATH="$ESPACE_TRAVAIL/.fosa_python${PYTHONPATH:+:$PYTHONPATH}"
fi
set -u

# Valeur lue dans la configuration active (profil compris) : le robot est sur
# le domaine 0 ; un ROS_DOMAIN_ID déjà exporté reste prioritaire.
lire_config() {
    python3 -c "from caytu_config import load_config; c = load_config(); print($1)"
}
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    export ROS_DOMAIN_ID="$(lire_config c.system.ros_domain_id)"
fi

verifier_paquet() {
    local paquet="$1"
    if ! ros2 pkg prefix "$paquet" >/dev/null 2>&1; then
        echo "Paquet ROS '$paquet' introuvable. Terminez d'abord ./install.sh." >&2
        exit 1
    fi
}

filtrer_erreurs() {
    awk 'tolower($0) ~ /error|fatal|failed|exception|unable|could not|erreur|échec/ { print; fflush() }'
}

arreter_simulation() {
    if [[ -n "${PID_SIMULATION:-}" ]] && kill -0 "$PID_SIMULATION" 2>/dev/null; then
        echo "Arrêt de la simulation PARC..."
        kill -INT -- "-$PID_SIMULATION" 2>/dev/null || true
        wait "$PID_SIMULATION" 2>/dev/null || true
    fi
}

case "$mode" in
    course)
        verifier_paquet caytu_course
        DONNEES="${FOSA_DONNEES:-$HOME/.ros/fosa}"
        mkdir -p "$DONNEES"
        JOURNAL="$DONNEES/course_$(date +%Y%m%d_%H%M%S).log"
        echo "Course : profil ${FOSA_PROFIL:-robot}, ROS_DOMAIN_ID=$ROS_DOMAIN_ID."
        echo "Le robot attend l'ordre puis le départ (./run.sh depart ou « allez »)."
        # Détachée de la session SSH : une coupure Wi-Fi n'arrête pas la course.
        INTERNE=(env "FOSA_PROFIL=${FOSA_PROFIL:-robot}" "FOSA_BASE_WS=${FOSA_BASE_WS:-}"
            "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" "$DOSSIER_SCRIPT/run.sh" course-premier-plan "$@")
        if command -v tmux >/dev/null 2>&1; then
            tmux new-session -d -s fosa_course "$(printf '%q ' "${INTERNE[@]}") 2>&1 | tee $(printf '%q' "$JOURNAL")"
            echo "Session tmux « fosa_course » (Ctrl-b d pour la quitter sans l'arrêter)."
            echo "Journal : $JOURNAL"
            if [[ -t 0 ]]; then exec tmux attach -t fosa_course; fi
        else
            setsid nohup "${INTERNE[@]}" > "$JOURNAL" 2>&1 < /dev/null &
            echo "Course lancée en arrière-plan (PID $!). Journal : $JOURNAL"
            echo "Ctrl-C quitte l'affichage sans arrêter la course (./run.sh arreter)."
            if [[ -t 0 ]]; then exec tail -n +1 -f "$JOURNAL"; fi
        fi
        ;;
    course-premier-plan)
        ros2 launch caytu_course course.launch.py "$@"
        ;;
    arreter)
        if command -v tmux >/dev/null 2>&1 && tmux has-session -t fosa_course 2>/dev/null; then
            tmux send-keys -t fosa_course C-c
        else
            pkill -INT -f "ros2 launch caytu_course course.launch.py" \
                || echo "Aucune course en cours."
        fi
        ;;
    cartographie)
        verifier_paquet caytu_course
        ros2 launch caytu_course course.launch.py mission:=false "$@"
        ;;
    depart)
        topic="$(lire_config c.course.start_topic)"
        # Attend que la course écoute (10 s au plus) : le départ n'est pas perdu.
        ros2 topic pub --once -w 1 --max-wait-time-secs 10 \
            "$topic" std_msgs/msg/String "{data: commande}"
        ;;
    ordre-test)
        ordre="${1:-}"
        motif_ordre='^[][0-9 ,]+$'     # chiffres, virgules, espaces, crochets
        if [[ -z "$ordre" || ! "$ordre" =~ $motif_ordre ]]; then
            echo "Usage : ./run.sh ordre-test \"7,1,9,4\"  (essais seulement)" >&2
            exit 2
        fi
        echo "ATTENTION : en course, l'ordre vient UNIQUEMENT des organisateurs."
        topic="$(lire_config c.course.order_topic)"
        ros2 topic pub -t 5 -r 1 --qos-reliability reliable \
            --qos-durability transient_local "$topic" std_msgs/msg/String "{data: '$ordre'}"
        ;;
    etat)
        ros2 topic echo "$(lire_config c.course.state_topic)" std_msgs/msg/String
        ;;
    sauver-carte)
        nom="${1:-arene_$(date +%H%M%S)}"
        dossier="${FOSA_DONNEES:-$HOME/.ros/fosa}/cartes"
        mkdir -p "$dossier"
        ros2 service call /slam_toolbox/serialize_map \
            slam_toolbox/srv/SerializePoseGraph "{filename: '$dossier/$nom'}"
        echo "Carte : $dossier/$nom.posegraph (course.slam_map_file si autorisé)."
        ;;
    diagnostic-robot)
        "$DOSSIER_SCRIPT/outils/diagnostic_robot.sh" "$@"
        ;;
    complet)
        verifier_paquet parc_robot_bringup
        echo "Démarrage du monde PARC. Ctrl+C arrête l'assistant et la simulation."
        setsid ros2 launch parc_robot_bringup task.launch.py \
            > >(filtrer_erreurs) 2>&1 &
        PID_SIMULATION=$!
        trap arreter_simulation EXIT INT TERM
        if ! timeout 120s ros2 topic echo /clock --once >/dev/null 2>&1; then
            echo "Le monde PARC n'a pas publié /clock dans le délai prévu." >&2
            exit 1
        fi
        echo "Simulation prête. Démarrage de l'assistant vocal."
        ros2 launch caytu_assistant assistant.launch.py "$@"
        ;;
    navigation)
        verifier_paquet caytu_nav_solution
        verifier_paquet parc_robot_bringup
        echo "La simulation PARC doit déjà tourner dans un autre terminal."
        echo "Le moniteur affichera la progression; les résultats seront enregistrés dans results/."
        readarray -t CONFIG_NAVIGATION < <(python3 - "$DOSSIER_SCRIPT/caytu_config/config/fosa.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as fichier:
    navigation = yaml.safe_load(fichier)["navigation"]
for cle in ("controller", "localization", "profile"):
    print(navigation[cle])
PY
)
        ros2 run caytu_nav_solution fosa_benchmark --ros-args \
            -p start_simulation:=false \
            -p controller:="${CONFIG_NAVIGATION[0]}" \
            -p localization:="${CONFIG_NAVIGATION[1]}" \
            -p profile:="${CONFIG_NAVIGATION[2]}" \
            -p results_dir:="$DOSSIER_SCRIPT/results" "$@"
        ;;
    assistant)
        ros2 launch caytu_assistant assistant.launch.py "$@"
        ;;
    tts)
        texte="${*:-}"
        if [[ -z "$texte" ]]; then
            if [[ -t 0 ]]; then
                read -r -p "Texte à prononcer : " texte
            else
                echo "Usage : ./run.sh tts \"texte à prononcer\"" >&2
                exit 2
            fi
        fi
        python3 - "$texte" <<'PY'
import sys
import rclpy
from std_msgs.msg import String
from caytu_assistant.sortie_vocale import NoeudSortieVocale

rclpy.init()
noeud = NoeudSortieVocale()
try:
    noeud._parler(String(data=sys.argv[1]))
finally:
    noeud.destroy_node()
    rclpy.shutdown()
PY
        ;;
    gazebo)
        verifier_paquet parc_robot_bringup
        echo "Simulation PARC active. Seules les erreurs seront affichées. Ctrl+C arrête le monde."
        ros2 launch parc_robot_bringup task.launch.py "$@" 2>&1 | filtrer_erreurs
        ;;
    benchmark)
        ros2 run caytu_nav_solution fosa_benchmark --ros-args \
            -p start_simulation:=false "$@"
        ;;
    compare)
        ros2 run caytu_nav_solution fosa_compare \
            --results-dir "$DOSSIER_SCRIPT/results" "$@"
        ;;
    diagnostic)
        python3 "$DOSSIER_SCRIPT/tools/diag_nav.py" "$@"
        ;;
    help|-h|--help)
        afficher_aide
        ;;
    *)
        echo "Mode inconnu : $mode" >&2
        afficher_aide >&2
        exit 2
        ;;
esac
