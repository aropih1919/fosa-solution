#!/usr/bin/env bash
# Lance un mode de la course Fosa depuis un terminal préparé.
set -euo pipefail

DOSSIER_SCRIPT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ESPACE_TRAVAIL="$(cd -- "$DOSSIER_SCRIPT/../.." && pwd)"
ROS_DISTRO="jazzy"

afficher_aide() {
    cat <<'AIDE'
Usage : ./run.sh [MODE] [arguments ROS]

Modes (robot réel par défaut) :
  preparation   les 15 min avant le match : nouvelle carte, recherche et
                mesure des 4 tags, enregistrement (exploration autonome)
  preparation-manuelle
                carte et tags enregistrés pendant que vous guidez le robot
  course        le match : se localise sur la carte de la préparation (ou
                cartographie en direct), attend l'ordre puis le départ
  suivi         afficher en phrases ce que fait le robot (tags, plans, sauts…)
  arreter       arrêter la préparation ou la course
  depart        donner le départ par commande (la course doit attendre)
  ordre-test O  publier un ordre de test, ex. "7,1,9,4" (ESSAIS SEULEMENT)
  etat          état brut de la course
  cartographie  SLAM, Nav2 et perception sans mission (réglages)
  sauver-carte  enregistrer la carte pendant que SLAM tourne, avant l'arrêt
  diagnostic-robot  vérifier topics, TF, caméras, audio, arbitre (1er créneau)
  mesure-rotation   deux tours lents sur place : odométrie contre gyromètre
                    et lidar (robot dégagé, course arrêtée, ~1 min)

Profils : FOSA_PROFIL=simulation pour Gazebo (sinon robot réel).
Espace de travail de base du robot (parc_msgs de l'arbitre) : FOSA_BASE_WS,
~/ros2_ws par défaut (sourcé avant le nôtre).

Exemples :
  ./run.sh preparation      puis, une fois terminée :   ./run.sh course
  ./run.sh preparation-manuelle, puis ./run.sh sauver-carte avant ./run.sh arreter
  ./run.sh suivi            (dans un second terminal)
  ./run.sh depart
AIDE
}

mode="${1:-}"
if [[ -z "$mode" ]]; then
    if [[ -t 0 ]]; then
        echo "Que souhaitez-vous lancer ?"
        select choix in "préparation (carte + tags)" "course (match)" "suivi" "départ par commande" "diagnostic robot" "quitter"; do
            case "$REPLY" in
                1) mode="preparation"; break ;;
                2) mode="course"; break ;;
                3) mode="suivi"; break ;;
                4) mode="depart"; break ;;
                5) mode="diagnostic-robot"; break ;;
                6) exit 0 ;;
                *) echo "Choisissez un numéro de 1 à 6." ;;
            esac
        done
    else
        afficher_aide >&2
        exit 2
    fi
fi
if (($#)); then shift; fi

if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    echo "ROS 2 Jazzy est absent : installez-le avant de lancer la course." >&2
    exit 1
fi
if [[ ! -f "$ESPACE_TRAVAIL/install/setup.bash" ]]; then
    echo "Workspace non compilé : $ESPACE_TRAVAIL/install/setup.bash est absent." >&2
    echo "Lancez ./deployer_robot.sh (robot) ou colcon build --packages-up-to caytu_course." >&2
    exit 1
fi

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
# Espace de travail de base du robot (paquets des organisateurs, dont
# parc_msgs pour l'arbitre) : sourcé avant le nôtre, qui reste prioritaire.
export FOSA_BASE_WS="${FOSA_BASE_WS:-$HOME/ros2_ws}"
if [[ "$(cd "$FOSA_BASE_WS" 2>/dev/null && pwd)" != "$ESPACE_TRAVAIL" ]]; then
    for script in "$FOSA_BASE_WS/install/setup.bash" "$FOSA_BASE_WS/setup.bash"; do
        if [[ -f "$script" ]]; then source "$script"; break; fi
    done
fi
source "$ESPACE_TRAVAIL/install/setup.bash"
export FOSA_CONFIG_DIR="$DOSSIER_SCRIPT/caytu_config/config"
case "${FOSA_PROFIL:-robot}" in
    robot)
        # Une surcharge héritée d'une simulation ne doit jamais configurer le robot réel.
        unset FOSA_CONFIG_OVERRIDE ;;
    simulation)
        export FOSA_CONFIG_OVERRIDE="$FOSA_CONFIG_DIR/profils/simulation.yaml" ;;
    *) echo "Profil inconnu : $FOSA_PROFIL (robot | simulation)" >&2; exit 2 ;;
esac
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
# Le profil actif détermine le domaine : ne pas réutiliser par accident celui
# d'une session précédente (robot réel ou Gazebo).
export ROS_DOMAIN_ID="$(lire_config c.system.ros_domain_id)"

verifier_arbitre() {
    if ! python3 -c 'import parc_msgs.msg' 2>/dev/null; then
        echo "ATTENTION : parc_msgs introuvable (FOSA_BASE_WS=$FOSA_BASE_WS)." >&2
        echo "Les visites ne seront PAS signalées à l'arbitre." >&2
    fi
}

# Lance la course (match ou préparation) détachée de la session SSH : une
# coupure Wi-Fi ne l'arrête pas. Journal conservé dans ~/.ros/fosa.
lancer_detache() {
    local nom="$1"; shift
    local donnees="${FOSA_DONNEES:-$HOME/.ros/fosa}" journal
    mkdir -p "$donnees"
    journal="$donnees/${nom}_$(date +%Y%m%d_%H%M%S).log"
    local interne=(env "FOSA_PROFIL=${FOSA_PROFIL:-robot}" "FOSA_BASE_WS=${FOSA_BASE_WS:-}"
        "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" "$DOSSIER_SCRIPT/run.sh" course-premier-plan "$@")
    echo "Suivi lisible dans un autre terminal : ./run.sh suivi"
    if command -v tmux >/dev/null 2>&1; then
        tmux new-session -d -s fosa_course "$(printf '%q ' "${interne[@]}") 2>&1 | tee $(printf '%q' "$journal")"
        echo "Session tmux « fosa_course » (Ctrl-b d pour la quitter sans l'arrêter)."
        echo "Journal : $journal"
        if [[ -t 0 ]]; then exec tmux attach -t fosa_course; fi
    else
        setsid nohup "${interne[@]}" > "$journal" 2>&1 < /dev/null &
        echo "Lancé en arrière-plan (PID $!). Journal : $journal"
        echo "Ctrl-C quitte l'affichage sans arrêter (./run.sh arreter)."
        if [[ -t 0 ]]; then exec tail -n +1 -f "$journal"; fi
    fi
}

verifier_paquet() {
    local paquet="$1"
    if ! ros2 pkg prefix "$paquet" >/dev/null 2>&1; then
        echo "Paquet ROS '$paquet' introuvable. Déployez d'abord la solution (./deployer_robot.sh)." >&2
        exit 1
    fi
}

case "$mode" in
    course)
        verifier_paquet caytu_course
        verifier_arbitre
        echo "Match : profil ${FOSA_PROFIL:-robot}, ROS_DOMAIN_ID=$ROS_DOMAIN_ID."
        python3 -c "
from caytu_config import load_config
from caytu_course.lancement import carte_preparee, fichier_tags
c = load_config(); carte = carte_preparee(c)
if carte is None:
    print('Aucune préparation : carte construite en direct, tags à chercher.')
else:
    print(f'Préparation utilisée : {carte}.posegraph et {fichier_tags(c)}.')
    print('Le robot doit partir de la position où la préparation a commencé.')"
        echo "Le robot attend l'ordre puis le départ (./run.sh depart ou « allez »)."
        lancer_detache course "$@"
        ;;
    preparation)
        verifier_paquet caytu_course
        echo "Préparation : robot posé à la POSITION DE DÉPART du match, arène dégagée."
        echo "Le robot explore seul, mesure les tags et enregistre tout (12 min au plus)."
        lancer_detache preparation mode:=preparation "$@"
        ;;
    preparation-manuelle)
        verifier_paquet caytu_course
        echo "Préparation guidée : utilisez un seul moyen de pilotage manuel sur /cmd_vel."
        echo "SLAM enregistre la carte et les caméras mémorisent les tags observés."
        echo "Avant l'arrêt, exécutez ./run.sh sauver-carte pendant que SLAM tourne ; les tags sont écrits en continu."
        echo "Aucun objectif autonome n'est lancé."
        lancer_detache preparation-manuelle mode:=preparation_manuelle "$@"
        ;;
    suivi)
        ros2 run caytu_course suivi_course "$@"
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
        ros2 launch caytu_course course.launch.py mode:=cartographie "$@"
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
        echo "Ne pas utiliser si l'arbitre tourne : armer la course dans son tableau de bord."
        topic="$(lire_config c.course.order_topic)"
        liste="${ordre//,/ }"          # 7,1,9,4 -> 7 1 9 4
        ros2 topic pub -t 5 -r 1 --qos-reliability reliable \
            --qos-durability transient_local "$topic" parc_msgs/msg/TagOrder \
            "{tag_ids: [${liste//[\[\]]/}]}"
        ;;
    etat)
        ros2 topic echo "$(lire_config c.course.state_topic)" std_msgs/msg/String
        ;;
    sauver-carte)
        dossier="$(lire_config 'c.course.preparation.directory')"
        dossier="${dossier/#\~/$HOME}"
        mkdir -p "$dossier"
        base="$dossier/carte"
        reponse="$(ros2 service call /slam_toolbox/serialize_map \
            slam_toolbox/srv/SerializePoseGraph "{filename: '$base'}" 2>&1)" || {
            printf '%s\n' "$reponse" >&2
            echo "Échec : SLAM doit encore tourner. Ne l'arrêtez qu'après cette sauvegarde." >&2
            exit 1
        }
        printf '%s\n' "$reponse"
        if [[ ! -s "$base.posegraph" || ! -s "$base.data" ]]; then
            echo "Échec : les fichiers $base.posegraph et $base.data ne sont pas présents." >&2
            exit 1
        fi
        echo "Carte enregistrée : $base.posegraph."
        if [[ -s "$dossier/tags.yaml" ]]; then
            python3 -c 'import sys, yaml; d=yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}; print("Tags mémorisés :", ", ".join(map(str, d.get("tags", {}).keys())) or "aucun")' "$dossier/tags.yaml"
        else
            echo "ATTENTION : fichier tags.yaml absent; vérifiez que les caméras ont détecté les tags." >&2
        fi
        ;;
    diagnostic-robot)
        "$DOSSIER_SCRIPT/outils/diagnostic_robot.sh" "$@"
        ;;
    mesure-rotation)
        if pgrep -f "[c]aytu_course/course_parc|[c]ourse.launch.py" >/dev/null; then
            echo "La course tourne : ./run.sh arreter d'abord (une seule source de commandes)." >&2
            exit 1
        fi
        python3 "$DOSSIER_SCRIPT/outils/mesure_rotation.py" "$@"
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
