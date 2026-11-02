#!/usr/bin/env bash
# Lance un mode existant de la solution Fosa depuis un terminal préparé.
set -euo pipefail

DOSSIER_SCRIPT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ESPACE_TRAVAIL="$(cd -- "$DOSSIER_SCRIPT/../.." && pwd)"
ROS_DISTRO="jazzy"

afficher_aide() {
    cat <<'AIDE'
Usage : ./run.sh [MODE] [arguments ROS]

Modes disponibles :
  complet       Gazebo PARC et assistant vocal complet (mode par défaut)
  navigation    tâche officielle surveillée, avec résultats conservés
  assistant     assistant complet sur un robot ou simulateur déjà actif
  tts [TEXTE]   prononcer un texte avec le fournisseur TTS configuré
  gazebo        terminal 1 : lancer task.launch.py, en affichant les erreurs
                terminal 2 : ./run.sh navigation
  benchmark     ouvrir l'outil de benchmark sur un monde déjà lancé
  compare       comparer les rapports présents dans results/
  diagnostic    lancer tools/diag_nav.py

Exemples :
  ./run.sh
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
        select choix in "solution complète" "navigation officielle" "assistant seul" "test TTS" "benchmark" "quitter"; do
            case "$REPLY" in
                1) mode="complet"; break ;;
                2) mode="navigation"; break ;;
                3) mode="assistant"; break ;;
                4) mode="tts"; break ;;
                5) mode="benchmark"; break ;;
                6) exit 0 ;;
                *) echo "Choisissez un numéro de 1 à 6." ;;
            esac
        done
    else
        mode="complet"
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
source "$ESPACE_TRAVAIL/install/setup.bash"
export FOSA_CONFIG_DIR="${FOSA_CONFIG_DIR:-$DOSSIER_SCRIPT/caytu_config/config}"
if [[ -f "$ESPACE_TRAVAIL/.venv/bin/activate" ]]; then
    source "$ESPACE_TRAVAIL/.venv/bin/activate"
    DOSSIER_PAQUETS_PYTHON="$("$ESPACE_TRAVAIL/.venv/bin/python" -c \
        'import sys; print(f"python{sys.version_info.major}.{sys.version_info.minor}")')"
    DOSSIER_PAQUETS_PYTHON="$ESPACE_TRAVAIL/.venv/lib/$DOSSIER_PAQUETS_PYTHON/site-packages"
    if [[ -d "$DOSSIER_PAQUETS_PYTHON" ]]; then
        export PYTHONPATH="$DOSSIER_PAQUETS_PYTHON${PYTHONPATH:+:$PYTHONPATH}"
    fi
fi
set -u

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    export ROS_DOMAIN_ID="$(python3 - "$DOSSIER_SCRIPT/caytu_config/config/fosa.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as fichier:
    print(yaml.safe_load(fichier)["system"]["ros_domain_id"])
PY
)"
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
