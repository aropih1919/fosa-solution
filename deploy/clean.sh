#!/usr/bin/env bash
# Fin de créneau : récupère les cartes, arrête nos programmes et efface tout ce
# que la solution a mis sur le robot.
#
#   deploy/clean.sh              rapatrie puis efface (recommandé)
#   deploy/clean.sh --no-fetch   efface sans rapatrier
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

if [[ "${1:-}" != "--no-fetch" ]]; then
    "$DEPLOY_DIR/fetch.sh"
fi

echo "== Arrêt et effacement sur le robot"
# Le script part par l'entrée standard (bash -s) : sa ligne de commande ne
# contient aucun des motifs cherchés, il ne risque pas de s'arrêter lui-même.
robot_ssh bash -s -- "$ROBOT_DIR" <<'REMOTE'
set -uo pipefail
ROOT="$1"
# Nos nœuds (install/), les nœuds Nav2 et slam_toolbox (paramètres dans
# $ROOT/tmp), l'enregistrement (maps/) et nos lancements. Pas « $ROOT » seul :
# la ligne de commande de ce shell le contient.
PATTERN="$ROOT/ws/install|$ROOT/tmp/|$ROOT/maps|caytu_nav_bringup|caytu_nav_solution|fosa_slam_"
ours() { pgrep -f "$PATTERN" | grep -vx "$$" || true; }

# 1. Arrêt propre (Ctrl-C : le robot reçoit une vitesse nulle), puis forcé.
pids=$(ours)
if [[ -n "$pids" ]]; then
    kill -INT $pids 2>/dev/null || true
    for _ in $(seq 1 10); do sleep 1; [[ -z "$(ours)" ]] && break; done
    pids=$(ours); [[ -n "$pids" ]] && kill -KILL $pids 2>/dev/null || true
fi
tmux kill-session -t fosa 2>/dev/null || true

# 2. Effacement : notre dossier, et ce qui aurait été lancé hors de env.sh.
rm -rf "$ROOT"
rm -rf "$HOME/fosa_maps" /tmp/fosa_slam_* /tmp/fosa_map_* 2>/dev/null || true

# 3. Contrôle.
left=$(ours)
if [[ -e "$ROOT" || -n "$left" ]]; then
    echo "ATTENTION : il reste $ROOT ou des processus ($left)." >&2
    exit 1
fi
echo "Robot nettoyé : $ROOT effacé, aucun programme Fosa en cours."
REMOTE
