#!/usr/bin/env bash
# Ouvre un terminal sur le robot, avec l'environnement Fosa chargé.
#
# Sur le robot, une session tmux « fosa » garde les programmes en vie si le
# Wi-Fi coupe la connexion SSH (sans tmux, une coupure arrête le robot en
# pleine course). Relancer connect.sh se rattache à la même session.
#   Ctrl-b c  nouvelle fenêtre    Ctrl-b n  fenêtre suivante    Ctrl-b d  détacher
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

SHELL_CMD="bash --rcfile $ROBOT_DIR/env.sh -i"
robot_ssh -t "
if [ ! -f $ROBOT_DIR/env.sh ]; then echo 'Lancer d abord deploy/deploy.sh' >&2; exit 1; fi
if command -v tmux > /dev/null; then
    exec tmux new-session -A -s fosa '$SHELL_CMD' \\; set-option -t fosa default-command '$SHELL_CMD'
else
    echo 'tmux absent : une coupure SSH arrêtera les programmes lancés ici.' >&2
    exec $SHELL_CMD
fi"
