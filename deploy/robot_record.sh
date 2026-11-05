#!/usr/bin/env bash
# Mode à distance : enregistre les capteurs SUR le robot (pas de trous dus au
# Wi-Fi), avec la seule commande publique « ros2 bag ». Aucun fichier de
# l'équipe n'est envoyé : seules des données du robot sont écrites, en mémoire
# vive ($ROBOT_DIR/maps/enregistrements), puis rapatriées.
#
#   deploy/robot_record.sh start [nom] [--camera]
#   deploy/robot_record.sh stop     arrête et rapatrie (deploy/fetch.sh)
#
# En fin de créneau, deploy/clean.sh efface tout du robot.
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

ACTION="${1:-}"; shift || true
OUT_DIR="$ROBOT_DIR/maps/enregistrements"

case "$ACTION" in
start)
    NAME="arene"; CAMERA=0
    for arg in "$@"; do
        case "$arg" in
            --camera) CAMERA=1 ;;
            *) NAME="$arg" ;;
        esac
    done
    # Topics du profil real_remote (lidar, odométrie, IMU des caméras, TF).
    mapfile -t TOPICS < <(python3 - "$REPO_DIR" "$CAMERA" <<'PY'
import sys, yaml
repo, camera = sys.argv[1], sys.argv[2] == "1"
with open(f"{repo}/caytu_nav_bringup/config/robots/real_remote.yaml", encoding="utf-8") as f:
    p = yaml.safe_load(f)
print("\n".join(p["record_topics"] + (p["record_camera_topics"] if camera else [])))
PY
)
    OUT="$OUT_DIR/${NAME}_$(date +%Y%m%d-%H%M%S)"
    # ssh recolle ses arguments en une ligne : on les protège (%q), sinon une
    # valeur vide ou avec espaces décalerait tout.
    ARGS=$(printf '%q ' "$ROBOT_BASE_SETUP" "$OUT" "${ROBOT_ROS_ENV:-}" "${TOPICS[@]}")
    robot_ssh "bash -s -- $ARGS" <<'REMOTE'
BASE="$1"; OUT="$2"; EXTRA_ENV="$3"; shift 3   # pas de set -u : setup.bash de ROS ne le supporte pas
unset HISTFILE
mkdir -p "$(dirname "$OUT")"
ROOT="$(dirname "$(dirname "$(dirname "$OUT")")")"
mkdir -p "$ROOT/ros_log"; chmod 700 "$ROOT"
export ROS_LOG_DIR="$ROOT/ros_log"
source "$BASE"
[[ -n "$EXTRA_ENV" ]] && eval "export $EXTRA_ENV"
# Sans contrôle des tâches, un programme lancé en arrière-plan par un shell
# non interactif ignore SIGINT : « stop » ne pourrait pas l'arrêter proprement.
set -m
setsid nohup ros2 bag record -o "$OUT" -s mcap --storage-preset-profile zstd_fast \
    --topics "$@" > "$OUT.log" 2>&1 < /dev/null &
sleep 3
if pgrep -f "bag record -o $OUT" > /dev/null; then
    echo "Enregistrement en cours sur le robot : $OUT"
else
    echo "Échec du démarrage :" >&2; cat "$OUT.log" >&2; exit 1
fi
REMOTE
    ;;
stop)
    robot_ssh "bash -s -- $(printf '%q' "$OUT_DIR")" <<'REMOTE'
set -u
running() { pgrep -f "bag record -o $1" || true; }
pids=$(running "$1")
if [[ -z "$pids" ]]; then echo "Aucun enregistrement en cours."; exit 0; fi
# Arrêt propre (le fichier de métadonnées est écrit à l'arrêt), puis SIGTERM.
for sig in INT TERM; do
    kill -$sig $(running "$1") 2>/dev/null || true
    for _ in $(seq 1 10); do sleep 1; [[ -z "$(running "$1")" ]] && break; done
    [[ -z "$(running "$1")" ]] && break
done
if [[ -n "$(running "$1")" ]]; then
    kill -KILL $(running "$1") 2>/dev/null || true
    echo "Enregistreur arrêté de force : réparer avec « ros2 bag reindex <dossier> »." >&2
else
    echo "Enregistrement arrêté proprement."
fi
REMOTE
    "$DEPLOY_DIR/fetch.sh"
    ;;
*)
    echo "Usage : deploy/robot_record.sh start [nom] [--camera] | stop" >&2
    exit 1
    ;;
esac
