#!/usr/bin/env bash
# Rapatrie sur le portable les cartes et enregistrements faits sur le robot
# (dans robot_data/maps/, non versionné). Ne supprime rien sur le robot.
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

mkdir -p "$LOCAL_DATA/maps"
if ! robot_ssh test -d "$ROBOT_DIR/maps"; then
    echo "Rien à récupérer : $ROBOT_DIR/maps absent sur le robot."
    exit 0
fi
robot_rsync "$ROBOT:$ROBOT_DIR/maps/" "$LOCAL_DATA/maps/"
echo "Cartes et enregistrements copiés dans $LOCAL_DATA/maps :"
ls -1 "$LOCAL_DATA/maps"
