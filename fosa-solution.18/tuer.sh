#!/usr/bin/env bash
# Pourquoi le robot n'avance pas ? À lancer SUR LE ROBOT, dans notre dossier d'équipe :
#
#   bash ~/teams/fosa/src/fosa-solution/outils/diagnostic_mouvement.sh
#   options : --oui (pas de question)  --vitesse 0.3 (rad/s)  --duree 3 (s)
#             --sans-arret (ne coupe pas nos nœuds)  --equipe ~/teams/fosa
#             --arreter-autres (propose d'arrêter les nœuds d'une AUTRE équipe qui
#                               publient sur /cmd_vel : seulement avec l'accord des organisateurs)
#
# Étapes :
#   1. arrête NOS nœuds (session tmux fosa_course, course, Nav2 lancés depuis notre
#      dossier), jamais ceux des organisateurs ni des autres équipes ;
#   2. liste ce qui publie encore sur /cmd_vel, avec PID, équipe d'origine et heure
#      de lancement (nœuds restés d'une autre équipe, ou des organisateurs) ;
#   3. appelle /start_motor ;
#   4. montre l'environnement DDS des pilotes du robot (obstacle_avoidance) ;
#   5. fait TOURNER le robot sur place (0,3 rad/s, 3 s, sans avancer), deux fois :
#        A. environnement du robot (sans nos réglages DDS) ;
#        B. notre environnement (celui de ce terminal + notre workspace) ;
#      et mesure à chaque étage : nos commandes vues par obstacle_avoidance ?
#      relayées sur /botwheel_explorer/cmd_vel ? roues qui tournent (odométrie) ?
#   6. donne la conclusion.
# Le robot TOURNE pendant l'étape 5 : dégager l'espace autour de lui.

EQUIPE_DIR="$HOME/teams/fosa"
USER="${USER:-$(id -un)}"
VITESSE=0.3
DUREE=3
OUI=0
ARRET=1
ARRETER_AUTRES=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --oui) OUI=1; shift ;;
        --vitesse) VITESSE="$2"; shift 2 ;;
        --duree) DUREE="$2"; shift 2 ;;
        --sans-arret) ARRET=0; shift ;;
        --arreter-autres) ARRETER_AUTRES=1; shift ;;
        --equipe) EQUIPE_DIR="$2"; shift 2 ;;
        -h|--help) awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"; exit 0 ;;
        *) echo "Option inconnue : $1" >&2; exit 2 ;;
    esac
done

TMP_DIR="$EQUIPE_DIR/tmp"
mkdir -p "$TMP_DIR"
TEST_PY="$(mktemp "$TMP_DIR/test_mouvement_XXXX.py")"
trap 'rm -f "$TEST_PY"' EXIT
titre() { echo; echo "=== $*"; }
# Python du système, celui de ROS Jazzy : jamais celui d'un .venv (rclpy n'y est pas).
PY=/usr/bin/python3
[[ -x "$PY" ]] || PY=python3
# Le robot fait déjà tourner beaucoup de processus ROS : avec CycloneDDS en
# ParticipantIndex « auto », la limite par défaut (10) provoque « Failed to find a
# free participant index ». On la relève pour nos essais seulement.
DDS_INDEX='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>119</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
avec_index() {   # valeur actuelle de CYCLONEDDS_URI -> valeur avec la limite relevée
    case "$1" in *MaxAutoParticipantIndex*) echo "$1" ;; "") echo "$DDS_INDEX" ;; *) echo "$1,$DDS_INDEX" ;; esac
}

sourcer_ros() {
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    # shellcheck disable=SC1090
    [[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"
}

# ---------------------------------------------------------------- test Python
cat > "$TEST_PY" <<'PYTHON'
"""Publie une rotation sur /cmd_vel et mesure chaque étage. Code de sortie :
0 robot tourne, 10 aucun abonné ne voit nos commandes, 11 relais muet,
12 relais à zéro, 13 roues immobiles, 20 rclpy introuvable, 21 participant DDS refusé,
22 autre erreur ROS."""
import sys
import time

etiquette = sys.argv[3]
try:
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
except Exception as erreur:                       # venv, ROS non sourcé, mauvais Python
    print(f'[{etiquette}] rclpy introuvable avec {sys.executable} '
          f'(Python {sys.version.split()[0]}) : {erreur!r}')
    sys.exit(20)

vitesse, duree = float(sys.argv[1]), float(sys.argv[2])
try:
    rclpy.init()
    noeud = rclpy.create_node('fosa_test_mouvement')
except Exception as erreur:
    texte = repr(erreur)
    print(f'[{etiquette}] création du nœud ROS impossible : {texte[:300]}')
    sys.exit(21 if 'participant' in texte.lower() else 22)
pub = noeud.create_publisher(Twist, '/cmd_vel', 10)
base, odom = [], []


def attendre(secondes):
    fin = time.monotonic() + secondes
    while time.monotonic() < fin:
        rclpy.spin_once(noeud, timeout_sec=0.05)


attendre(2.5)                                     # découverte DDS
autres = sorted({i.node_name for i in noeud.get_publishers_info_by_topic('/cmd_vel')
                 if i.node_name != 'fosa_test_mouvement'})
abonnes = sorted({i.node_name for i in noeud.get_subscriptions_info_by_topic('/cmd_vel')})
types = dict(noeud.get_topic_names_and_types())
type_base = (types.get('/botwheel_explorer/cmd_vel') or ['?'])[0]
if 'TwistStamped' in type_base:
    noeud.create_subscription(TwistStamped, '/botwheel_explorer/cmd_vel',
                              lambda m: base.append(m.twist.angular.z), qos_profile_sensor_data)
elif 'Twist' in type_base:
    noeud.create_subscription(Twist, '/botwheel_explorer/cmd_vel',
                              lambda m: base.append(m.angular.z), qos_profile_sensor_data)
noeud.create_subscription(Odometry, '/botwheel_explorer/odom',
                          lambda m: odom.append(m.twist.twist.angular.z), qos_profile_sensor_data)
attendre(1.0)
vus = pub.get_subscription_count()                # abonnés qui ont VRAIMENT apparié notre publieur
print(f'[{etiquette}] abonnés de /cmd_vel : {abonnes or "aucun"} ; '
      f'appariés à NOTRE publieur : {vus}')
print(f'[{etiquette}] autres publieurs de /cmd_vel : {autres or "aucun"}')
print(f'[{etiquette}] /botwheel_explorer/cmd_vel : type {type_base}')
base.clear(); odom.clear()
commande = Twist()
commande.angular.z = vitesse
fin = time.monotonic() + duree
while time.monotonic() < fin:
    pub.publish(commande)
    attendre(0.1)
arret = Twist()
for _ in range(10):
    pub.publish(arret)
    attendre(0.05)
attendre(0.5)
max_base = max((abs(v) for v in base), default=0.0)
max_odom = max((abs(v) for v in odom), default=0.0)
print(f'[{etiquette}] relais /botwheel_explorer/cmd_vel : {len(base)} messages, max |w| = {max_base:.2f}')
print(f'[{etiquette}] odométrie : {len(odom)} messages, max |w| = {max_odom:.2f}')
seuil = 0.3 * abs(vitesse)
if vus == 0:
    code, texte = 10, "obstacle_avoidance ne voit PAS nos commandes (DDS : domaine, RMW ou config CycloneDDS)"
elif not base:
    code, texte = 11, "le relais ne publie rien sur /botwheel_explorer/cmd_vel (autre point d'entrée ? demander aux organisateurs)"
elif max_base < seuil:
    code, texte = 12, "obstacle_avoidance reçoit mais envoie ~0 : il voit un obstacle trop proche (éloigner le robot des meubles)"
elif max_odom < seuil:
    code, texte = 13, "la base reçoit la commande mais les roues ne tournent pas : moteurs (/start_motor), arrêt d'urgence, batterie"
else:
    code, texte = 0, "LE ROBOT TOURNE : la chaîne de commande marche"
print(f'[{etiquette}] => {texte}')
noeud.destroy_node()
rclpy.try_shutdown()
sys.exit(code)
PYTHON

# ---------------------------------------------------------------- 1. nos nœuds
titre "1. Arrêt de NOS nœuds (équipe : $EQUIPE_DIR)"
# Nos processus : lancés depuis notre dossier ou avec notre workspace sourcé.
# Jamais les shells (le vôtre compris), ni ce script, ni les autres utilisateurs.
nos_processus() {
    local p comm cmd
    for p in $(pgrep -u "$USER" .); do
        [[ "$p" == "$$" || "$p" == "$PPID" ]] && continue
        comm=$(cat "/proc/$p/comm" 2> /dev/null) || continue
        case "$comm" in bash|sh|dash|zsh|sshd*|tmux*|sudo|su|login|screen|systemd|sd-pam) continue ;; esac
        cmd=$({ tr '\0' ' ' < "/proc/$p/cmdline"; } 2> /dev/null)
        [[ "$cmd" == *diagnostic_mouvement* || "$cmd" == *test_mouvement_* ]] && continue
        # Pas de motif « caytu_ » : les pilotes du robot (caytu_controller...) en ont aussi.
        if [[ "$cmd" == *course_parc* || "$cmd" == *"$EQUIPE_DIR/"* ]] \
           || { tr '\0' '\n' < "/proc/$p/environ"; } 2> /dev/null | grep -q "$EQUIPE_DIR/install"; then
            echo "$p"
        fi
    done
}
if (( ARRET )); then
    for session in $(tmux ls -F '#S' 2> /dev/null | grep -i '^fosa'); do
        tmux kill-session -t "$session" && echo "  session tmux $session fermée"
    done
    autres_sessions=$(tmux ls -F '#S' 2> /dev/null | grep -vi '^fosa' | tr '\n' ' ')
    [[ -n "$autres_sessions" ]] && echo "  sessions tmux d'autres équipes sur ce compte (non touchées) : $autres_sessions"
    liste=$(nos_processus)
    if [[ -n "$liste" ]]; then
        echo "  arrêt de $(echo "$liste" | wc -l) processus :"
        for p in $liste; do echo "    $p $(tr '\0' ' ' < "/proc/$p/cmdline" 2> /dev/null | cut -c1-110)"; done
        kill -INT $liste 2> /dev/null; sleep 4
        liste=$(nos_processus); [[ -n "$liste" ]] && { kill -KILL $liste 2> /dev/null; sleep 1; }
    fi
    [[ -z "$(nos_processus)" ]] && echo "  plus aucun processus de l'équipe" \
                                || echo "  ATTENTION : des processus de l'équipe résistent : $(nos_processus | tr '\n' ' ')"
else
    echo "  (--sans-arret : rien n'est arrêté)"
fi

# ---------------------------------------------------------------- 2-4. état
sourcer_ros
titre "2. Publieurs restants sur /cmd_vel (à qui appartiennent-ils ?)"
# Nœuds qui PUBLIENT sur /cmd_vel (le seul abonné attendu : obstacle_avoidance).
mapfile -t PUBLIEURS < <(timeout 15 ros2 topic info /cmd_vel -v 2> /dev/null \
    | grep -E "Node name|Endpoint type" | paste - - | grep PUBLISHER \
    | sed 's/.*Node name: *\([^ \t]*\).*/\1/' | sort -u)
AUTRES_PIDS=()
if ! command -v ros2 > /dev/null; then
    echo "  ros2 introuvable : ROS Jazzy non sourcé (source /opt/ros/jazzy/setup.bash)"
elif (( ${#PUBLIEURS[@]} == 0 )); then
    echo "  aucun publieur : la voie est libre"
fi
for noeud in "${PUBLIEURS[@]}"; do
    # Processus de ce nœud : « __node:=nom » (lancé par un launch) ou nom dans la commande.
    pids=$(pgrep -f -- "__node:=$noeud( |$)"; pgrep -f -- "/$noeud( |$)")
    pids=$(echo "$pids" | grep -v "^$" | sort -u)
    if [[ -z "$pids" ]]; then
        echo "  $noeud : processus introuvable (autre utilisateur, ou autre machine sur le réseau)"
        continue
    fi
    for p in $pids; do
        proprio=$(ps -o user= -p "$p" 2> /dev/null)
        env_p=$({ tr '\0' '\n' < "/proc/$p/environ"; } 2> /dev/null)
        # Origine : le dossier d'équipe présent dans son environnement (workspace sourcé).
        equipe=$(echo "$env_p" | grep -oE "$HOME/teams/[^/:]+" | head -1)
        [[ -z "$equipe" ]] && equipe=$({ tr '\0' ' ' < "/proc/$p/cmdline"; } 2> /dev/null \
                                       | grep -oE "$HOME/teams/[^/ ]+" | head -1)
        demarre=$(ps -o lstart= -p "$p" 2> /dev/null)
        if [[ -n "$equipe" ]]; then
            origine="équipe ${equipe#"$HOME/teams/"}"
        elif [[ -z "$env_p" ]]; then
            origine="origine illisible (utilisateur $proprio)"
        else
            origine="hors dossier d'équipe (organisateurs ?)"
        fi
        echo "  $noeud : PID $p, $origine, lancé le $demarre"
        # Candidats à l'arrêt : processus d'une AUTRE équipe uniquement.
        if [[ -n "$equipe" && "$equipe" != "$EQUIPE_DIR" ]]; then AUTRES_PIDS+=("$p"); fi
    done
done
if (( ${#AUTRES_PIDS[@]} )); then
    echo "  -> ${#AUTRES_PIDS[@]} processus d'une AUTRE équipe commandent le robot pendant notre créneau."
    if (( ARRETER_AUTRES )); then
        read -r -p "  Les arrêter (avec l'accord des organisateurs) ? [o/N] " reponse
        if [[ "$reponse" == [oO] ]]; then
            kill -INT "${AUTRES_PIDS[@]}" 2> /dev/null; sleep 3
            kill -KILL "${AUTRES_PIDS[@]}" 2> /dev/null
            echo "  arrêtés."
        fi
    else
        echo "     Montrer cette liste aux organisateurs ; pour les arrêter avec leur accord :"
        echo "     relancer avec --arreter-autres."
    fi
fi

titre "3. Activation des moteurs (/start_motor)"
if timeout 15 ros2 service list 2> /dev/null | grep -q '^/start_motor$'; then
    timeout 15 ros2 service call /start_motor std_srvs/srv/Empty > /dev/null 2>&1 \
        && echo "  /start_motor appelé" || echo "  /start_motor : échec ou pas de réponse"
else
    echo "  service /start_motor absent"
fi

titre "4. Environnement DDS des pilotes du robot"
pid=$(pgrep -f obstacle_avoidance | head -1)
PILOTES=()
if [[ -n "$pid" && -r "/proc/$pid/environ" ]]; then
    mapfile -t PILOTES < <(tr '\0' '\n' < "/proc/$pid/environ" \
        | grep -E '^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_LOCALHOST_ONLY|ROS_AUTOMATIC_DISCOVERY_RANGE)=')
    for v in "${PILOTES[@]}"; do echo "  pilotes : $v"; done
    (( ${#PILOTES[@]} )) || echo "  pilotes : aucune variable ROS/DDS (réglages par défaut)"
else
    echo "  environnement d'obstacle_avoidance illisible : l'essai A prend le domaine et le RMW de ce terminal"
    PILOTES=("RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}")
fi
echo "  ce terminal : ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-} CYCLONEDDS_URI=${CYCLONEDDS_URI:-}"
[[ -n "${VIRTUAL_ENV:-}" ]] && echo "  ATTENTION : .venv actif ($VIRTUAL_ENV) : les essais utilisent $PY, pas le Python du .venv"
# Essai A : limite de participants relevée si les pilotes sont en CycloneDDS.
uri_pilotes=""; rmw_pilotes=""
for v in "${PILOTES[@]}"; do
    case "$v" in CYCLONEDDS_URI=*) uri_pilotes="${v#CYCLONEDDS_URI=}" ;; RMW_IMPLEMENTATION=*) rmw_pilotes="${v#RMW_IMPLEMENTATION=}" ;; esac
done
if [[ "$rmw_pilotes" == rmw_cyclonedds_cpp || -n "$uri_pilotes" ]]; then
    mapfile -t PILOTES < <(printf '%s\n' "${PILOTES[@]}" | grep -v '^CYCLONEDDS_URI=')
    PILOTES+=("CYCLONEDDS_URI=$(avec_index "$uri_pilotes")")
fi

# ---------------------------------------------------------------- 5. essais
titre "5. Essai de rotation sur place ($VITESSE rad/s pendant $DUREE s)"
if (( ! OUI )); then
    read -r -p "  Le robot va TOURNER sur place. Espace dégagé ? [o/N] " reponse
    [[ "$reponse" == [oO] ]] || { echo "  Essai annulé."; exit 0; }
fi

echo "  -- A. environnement du robot (sans nos réglages)"
env -i HOME="$HOME" USER="$USER" LANG=C.UTF-8 TERM="${TERM:-xterm}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin "${PILOTES[@]}" \
    bash -c 'set +u; source /opt/ros/jazzy/setup.bash;
             [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash;
             "$3" "$0" "$1" "$2" A' "$TEST_PY" "$VITESSE" "$DUREE" "$PY"
CODE_A=$?
sleep 1

echo "  -- B. notre environnement (ce terminal + $EQUIPE_DIR/install)"
(
    set +u
    # shellcheck disable=SC1091
    [[ -f "$EQUIPE_DIR/install/setup.bash" ]] && source "$EQUIPE_DIR/install/setup.bash"
    unset VIRTUAL_ENV PYTHONHOME
    if [[ "${RMW_IMPLEMENTATION:-}" == rmw_cyclonedds_cpp || -n "${CYCLONEDDS_URI:-}" ]]; then
        export CYCLONEDDS_URI="$(avec_index "${CYCLONEDDS_URI:-}")"
    fi
    "$PY" "$TEST_PY" "$VITESSE" "$DUREE" B
)
CODE_B=$?

# ---------------------------------------------------------------- 6. conclusion
titre "6. Conclusion"
if (( CODE_A == 20 || CODE_B == 20 )); then
    echo "  rclpy introuvable : ROS Jazzy non sourcé ou mauvais Python (.venv). Désactiver le .venv"
    echo "  (deactivate), puis : source /opt/ros/jazzy/setup.bash ; utiliser /usr/bin/python3."
fi
if (( CODE_A == 21 || CODE_B == 21 )); then
    echo "  Participant DDS refusé (trop de processus ROS sur le robot) : exporter avant chaque lancement"
    echo "  CYCLONEDDS_URI=\"\${CYCLONEDDS_URI:+\$CYCLONEDDS_URI,}$DDS_INDEX\""
fi
if (( CODE_A == 22 || CODE_B == 22 )); then
    echo "  Erreur ROS à la création du nœud : voir la ligne [A] ou [B] ci-dessus."
fi
if (( CODE_A >= 20 && CODE_B >= 20 )); then
    :
elif (( CODE_A == 0 && CODE_B == 0 )); then
    echo "  Le robot tourne dans les deux environnements. La cause était nos propres nœuds"
    echo "  (publieurs concurrents sur /cmd_vel) ou les moteurs non activés."
    echo "  Avant chaque essai : ce script (ou au moins l'étape 1 et /start_motor)."
elif (( CODE_A == 0 && CODE_B != 0 )); then
    echo "  Le robot tourne SANS nos réglages mais PAS avec : notre environnement DDS est en cause."
    echo "  Lancer la solution sans notre CYCLONEDDS_URI (FOSA_DDS_ETENDU=0) ou avec celui des pilotes."
elif (( CODE_A == 10 && CODE_B == 10 )); then
    echo "  Personne n'apparie nos commandes : domaine ROS ou RMW différent des pilotes (voir étape 4)."
elif (( CODE_A == 12 || CODE_B == 12 )); then
    echo "  obstacle_avoidance bloque : éloigner le robot de tout meuble, puis relancer ce script."
elif (( CODE_A == 13 || CODE_B == 13 )); then
    echo "  Commande transmise mais roues immobiles : arrêt d'urgence, moteurs, batterie -> organisateurs."
elif (( CODE_A == 11 || CODE_B == 11 )); then
    echo "  Le relais ne publie rien : demander aux organisateurs le point d'entrée des vitesses."
else
    echo "  Résultats A=$CODE_A B=$CODE_B : voir les lignes [A] et [B] ci-dessus."
fi