#!/usr/bin/env bash
# Lance un mode de la course Fosa depuis un terminal préparé.
set -euo pipefail

DOSSIER_SCRIPT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ESPACE_TRAVAIL="${FOSA_WS:-$(cd -- "$DOSSIER_SCRIPT/../.." && pwd)}"
source "$DOSSIER_SCRIPT/outils/config_exploitation.sh"
ROS_DISTRO="$(lire_exploitation ros_distro)"
# Une valeur explicitement demandée par l'opérateur ne doit pas être écrasée
# par la bascule automatique « aucune préparation -> sans carte ».
NAVIGATION_DEMANDEE="${FOSA_NAVIGATION:-}"

afficher_aide() {
    cat <<'AIDE'
Usage : ./run.sh [MODE] [arguments ROS]

Modes (robot réel par défaut) :
  preparation   les 15 min avant le match : nouvelle carte, recherche et
                mesure des 4 tags, enregistrement (exploration autonome)
  preparation-manuelle
                carte et tags enregistrés pendant que vous guidez le robot
  course        le match : se localise sur la carte de la préparation ;
                sans préparation valide, bascule automatiquement en sans carte
  course-sans-carte  match autonome en odom, sans carte enregistrée ni SLAM
  suivi         afficher en phrases ce que fait le robot (tags, plans, sauts…)
  arreter       arrêter la préparation, la course et l'ancienne session tmux
  depart        donner le départ par commande (la course doit attendre)
  ordre-test O  publier un ordre de test, ex. "7,1,9,4" (ESSAIS SEULEMENT)
  test O        alias de ordre-test : publie l'ordre, ne lance pas la course
  etat          état brut de la course
  cartographie  SLAM, Nav2 et perception sans mission (réglages)
  sauver-carte  enregistrer la carte pendant que SLAM tourne, avant l'arrêt
  diagnostic-robot  vérifier topics, TF, caméras, audio, arbitre (1er créneau)
  diagnostic-dds    vérifier les imports et la création d'un nœud ROS
  mesure-rotation   deux tours lents sur place : odométrie contre gyromètre
                    et lidar (robot dégagé, course arrêtée, ~1 min)

Profils : FOSA_PROFIL=simulation pour Gazebo (sinon robot réel).
Carte : celle de la livraison, ou FOSA_CARTE=/chemin/carte pour forcer.
Espace de travail de base du robot (parc_msgs de l'arbitre) : FOSA_BASE_WS,
~/parc_ws en priorité puis ~/ros2_ws (sourcé avant le nôtre).
CycloneDDS saturé : FOSA_DDS_ETENDU=1 active la surcharge de dépannage
dans caytu_config/config/cyclonedds.xml (voir README, section F.1).

Exemples :
  ./run.sh preparation      puis, une fois terminée :   ./run.sh course
  ./run.sh preparation-manuelle, puis ./run.sh sauver-carte avant ./run.sh arreter
  ./run.sh suivi            (dans un second terminal)
  ./run.sh depart

Essai local sans arbitre (aucun ranking) :
  ./run.sh test '7,1,9,4'
  ./run.sh course-sans-carte
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

case "$mode" in
    help|-h|--help) afficher_aide; exit 0 ;;
    course-sans-carte) export FOSA_NAVIGATION=sans_carte; mode=course ;;
    test)
        echo "Mode test : publication d'un ordre local uniquement (aucun lancement, aucun ranking)." >&2
        mode=ordre-test ;;
    preparation|preparation-manuelle|cartographie) export FOSA_NAVIGATION=carte ;;
esac
export FOSA_NAVIGATION="${FOSA_NAVIGATION:-carte}"
if [[ "${FOSA_PROFIL:-robot}" == simulation ]]; then
    cache="$(lire_exploitation simulation.cache)"
    export FOSA_DONNEES="${FOSA_DONNEES:-${cache/#\~/$HOME}}"
else
    export FOSA_DONNEES="${FOSA_DONNEES:-$HOME/$(lire_exploitation dossier_robot)/donnees}"
fi
ETAT_SESSION="$FOSA_DONNEES/sessions/course.json"
# L'arrêt couvre la livraison active et les anciennes livraisons du robot.
PORTEE_ROBOT="$HOME/$(lire_exploitation dossier_robot)"
PORTEES_ARRET=("$ESPACE_TRAVAIL")
if [[ "$PORTEE_ROBOT" != "$ESPACE_TRAVAIL" ]]; then
    PORTEES_ARRET+=("$PORTEE_ROBOT")
fi
PORTEES_ARGUMENT=()
for portee in "${PORTEES_ARRET[@]}"; do
    PORTEES_ARGUMENT+=(--portee "$portee")
done

# Valeur lue dans la configuration active (profil compris) : le robot est sur
# le domaine 0 ; le profil impose aussi le domaine aux outils.
lire_config() {
    python3 -c "from caytu_config import load_config; c = load_config(); print($1)"
}

# L'ancienne version lançait la course dans tmux. Cette session connue doit
# être arrêtée avant la nouvelle, sinon deux nœuds publient sur /cmd_vel.
arreter_ancienne_session() {
    command -v tmux >/dev/null 2>&1 || return 0
    tmux has-session -t fosa_course 2>/dev/null || return 0
    echo "Arrêt de l'ancienne session tmux « fosa_course »." >&2
    tmux kill-session -t fosa_course
}

signaler_course_ancienne() {
    local lignes
    lignes="$(pgrep -af '[c]aytu_course/course_parc|[c]ourse\.launch\.py' || true)"
    [[ -z "$lignes" ]] && return 0
    echo "Des nœuds Fosa tournent encore :" >&2
    printf '%s\n' "$lignes" | sed 's/^/  /' >&2
    return 1
}

arreter_moteurs() {
    local service="${1:-/stop_motor}" reponse
    command -v ros2 >/dev/null 2>&1 || return 0
    if reponse="$(timeout 6 ros2 service call "$service" std_srvs/srv/Empty 2>&1)"; then
        echo "Moteurs coupés ($service)."
    else
        echo "Service $service indisponible : coupez les moteurs au bouton." >&2
    fi
}

demarrer_moteurs() {
    local service="${1:-/start_motor}"
    if timeout 6 ros2 service call "$service" std_srvs/srv/Empty >/dev/null 2>&1; then
        echo "Moteurs activés ($service)."
    else
        echo "Service $service indisponible : le robot ne bougera pas." >&2
    fi
}

annuler_vitesse() {
    command -v ros2 >/dev/null 2>&1 || return 0
    local topic publie=0
    for topic in /fosa/cmd_vel /cmd_vel; do
        if timeout 5 ros2 topic pub --once -w 1 --max-wait-time-secs 2 \
                "$topic" geometry_msgs/msg/Twist \
                "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
                >/dev/null 2>&1; then
            publie=1
        fi
    done
    ((publie)) && echo "Vitesse nulle publiée sur la commande du robot."
    return 0
}

# L'arrêt doit rester disponible même si ROS ou le workspace est endommagé.
if [[ "$mode" == arreter ]]; then
    arreter_ancienne_session
    source_arret_ok=0
    if [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
        set +u
        source "/opt/ros/$ROS_DISTRO/setup.bash"
        if [[ -f "$ESPACE_TRAVAIL/install/setup.bash" ]]; then
            source "$ESPACE_TRAVAIL/install/setup.bash" || true
        fi
        set -u
        export FOSA_CONFIG_DIR="$DOSSIER_SCRIPT/caytu_config/config"
        source "$DOSSIER_SCRIPT/outils/environnement_dds.sh"
        configurer_dds || true
        export ROS_DOMAIN_ID="$(lire_config c.system.ros_domain_id 2>/dev/null || echo 0)"
        source_arret_ok=1
    fi
    echec=0
    if ! python3 "$DOSSIER_SCRIPT/outils/processus.py" arreter \
            --etat "$ETAT_SESSION" "${PORTEES_ARGUMENT[@]}"; then
        echec=1
    fi
    if ((source_arret_ok)); then
        service_arret="$(lire_config c.course.motor_stop_service 2>/dev/null || echo /stop_motor)"
        arreter_moteurs "$service_arret"
        annuler_vitesse
    fi
    signaler_course_ancienne || echec=1
    if ((echec)); then
        echo "Arrêt incomplet : voir les processus ci-dessus." >&2
        exit 1
    fi
    echo "Arrêt terminé : course, nœuds et moteurs sont arrêtés."
    exit 0
fi

if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    echo "ROS 2 Jazzy est absent : installez-le avant de lancer la course." >&2
    exit 1
fi
if [[ ! -f "$ESPACE_TRAVAIL/install/setup.bash" ]]; then
    echo "Workspace non compilé : $ESPACE_TRAVAIL/install/setup.bash est absent." >&2
    echo "Lancez ./deployer_robot.sh --installer (robot) ou simulation/simulation.sh preparer (PC)." >&2
    exit 1
fi

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
# Espace de travail de base du robot (paquets des organisateurs, dont
# parc_msgs pour l'arbitre) : sourcé avant le nôtre, qui reste prioritaire.
# Les organisateurs ont annoncé ~/parc_ws ; ~/ros2_ws reste accepté.
base_ws_defaut="$HOME/parc_ws"
if [[ ! -f "$base_ws_defaut/install/setup.bash" \
        && -f "$HOME/ros2_ws/install/setup.bash" ]]; then
    base_ws_defaut="$HOME/ros2_ws"
fi
export FOSA_BASE_WS="${FOSA_BASE_WS:-$base_ws_defaut}"
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

source "$DOSSIER_SCRIPT/outils/environnement_dds.sh"
configurer_dds

# Le profil actif détermine le domaine : ne pas réutiliser par accident celui
# d'une session précédente (robot réel ou Gazebo).
export ROS_DOMAIN_ID="$(lire_config c.system.ros_domain_id)"

verifier_arbitre() {
    if ! python3 -c 'import parc_msgs.msg' 2>/dev/null; then
        echo "parc_msgs introuvable (FOSA_BASE_WS=$FOSA_BASE_WS)." >&2
        echo "Chargez le workspace organisateur : source ~/parc_ws/install/setup.bash" >&2
        exit 1
    fi
}

# Lance la course (match ou préparation) détachée de la session SSH : une
# coupure Wi-Fi ne l'arrête pas. Journal conservé dans FOSA_DONNEES.
lancer_detache() {
    local nom="$1"; shift
    local donnees="$FOSA_DONNEES" journal
    if python3 "$DOSSIER_SCRIPT/outils/processus.py" actif --etat "$ETAT_SESSION" >/dev/null 2>&1; then
        echo "Une session Fosa est déjà active : ./run.sh arreter, puis relancez." >&2
        echo "Si vous êtes simplement reconnecté après une coupure SSH, utilisez ./run.sh suivi." >&2
        exit 1
    fi
    arreter_ancienne_session
    # Nœuds d'une ancienne version ou d'une session sans état : les arrêter
    # automatiquement plutôt que de laisser deux sources de commandes.
    if python3 "$DOSSIER_SCRIPT/outils/processus.py" actif \
            --etat "$ETAT_SESSION" "${PORTEES_ARGUMENT[@]}" >/dev/null 2>&1; then
        echo "Anciens nœuds Fosa détectés : arrêt automatique avant le lancement."
        python3 "$DOSSIER_SCRIPT/outils/processus.py" arreter \
            --etat "$ETAT_SESSION" "${PORTEES_ARGUMENT[@]}" || exit 1
        annuler_vitesse
    fi
    mkdir -p "$donnees"
    journal="$donnees/${nom}_$(date +%Y%m%d_%H%M%S).log"
    local interne=("$DOSSIER_SCRIPT/run.sh" course-premier-plan "$@")
    echo "Suivi lisible dans un autre terminal : ./run.sh suivi"
    python3 "$DOSSIER_SCRIPT/outils/processus.py" lancer --etat "$ETAT_SESSION" \
        --journal "$journal" "${PORTEES_ARGUMENT[@]}" -- "${interne[@]}"
    echo "Lancement détaché de SSH. Arrêt : ./run.sh arreter ; journal : $journal"
}

verifier_paquet() {
    local paquet="$1"
    if ! ros2 pkg prefix "$paquet" >/dev/null 2>&1; then
        echo "Paquet ROS '$paquet' introuvable. Déployez d'abord avec ./deployer_robot.sh --installer." >&2
        exit 1
    fi
}

case "$mode" in
    course)
        verifier_paquet caytu_course
        verifier_arbitre
        if [[ -n "${FOSA_CARTE:-}" && "${FOSA_NAVIGATION:-carte}" == carte ]] \
                && ! python3 -c "
from caytu_config import load_config
from caytu_course.lancement import carte_preparee
raise SystemExit(0 if carte_preparee(load_config()) is not None else 1)"; then
            echo "Carte introuvable : FOSA_CARTE=$FOSA_CARTE" >&2
            exit 1
        fi
        # Sans carte valide, le mode sans carte est plus sûr que de
        # construire une carte pendant les 10 minutes du match.
        if [[ -z "$NAVIGATION_DEMANDEE" && "${FOSA_NAVIGATION:-carte}" == carte ]] \
                && ! python3 -c "
from caytu_config import load_config
from caytu_course.lancement import carte_preparee
raise SystemExit(0 if carte_preparee(load_config()) is not None else 1)"; then
            export FOSA_NAVIGATION=sans_carte
            echo "Aucune carte valide : bascule automatique en sans carte (odom)."
        fi
        echo "Match : profil ${FOSA_PROFIL:-robot}, ROS_DOMAIN_ID=$ROS_DOMAIN_ID."
        python3 -c "
from caytu_config import load_config
from caytu_course.lancement import carte_preparee, fichier_tags_match
c = load_config(); carte = carte_preparee(c)
if c.navigation.mode == 'sans_carte':
    print('Sans carte : costmaps roulantes en odom ; aucune carte chargée.')
elif carte is None:
    print('Aucune carte : cartographie en direct, tags à chercher.')
else:
    print(f'Carte utilisée : {carte}.posegraph et {fichier_tags_match(c)}.')
    print('Placer le robot à la position de départ de la carte avant le match.')"
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
        echo "Préparation guidée : pilotez sur $(lire_config c.navigation.securite.topic_entree)."
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
    cartographie)
        verifier_paquet caytu_course
        lancer_detache cartographie mode:=cartographie "$@"
        ;;
    depart)
        demarrer_moteurs "$(lire_config c.course.motor_start_service)"
        topic="$(lire_config c.course.start_topic)"
        # Attend que la course écoute (10 s au plus) : le départ n'est pas perdu.
        ros2 topic pub --once -w 1 --max-wait-time-secs 10 \
            "$topic" std_msgs/msg/String "{data: commande}"
        ;;
    ordre-test)
        ordre="${1:-}"
        liste="$(python3 - "$ordre" <<'PYTHON'
import sys
from caytu_config import load_config
from caytu_course.ordre_tags import lire_ordre
print(', '.join(map(str, lire_ordre(sys.argv[1], load_config().course.tag_ids))))
PYTHON
)"
        echo "ATTENTION : en course, l'ordre vient UNIQUEMENT des organisateurs."
        echo "Ne pas utiliser si l'arbitre tourne : armer la course dans son tableau de bord."
        topic="$(lire_config c.course.order_topic)"
        ros2 topic pub -t 5 -r 1 --qos-reliability reliable \
            --qos-durability transient_local "$topic" parc_msgs/msg/TagOrder \
            "{tag_ids: [$liste]}"
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
    diagnostic-dds)
        python3 "$DOSSIER_SCRIPT/outils/diagnostic_dds.py"
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
