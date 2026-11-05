#!/usr/bin/env bash
# Mode à distance : le portable peut-il piloter le robot par le réseau ?
#
#   deploy/check_link.sh        (sur le portable, ROS 2 Jazzy sourcé)
#
# Rien n'est envoyé ni écrit sur le robot. Vérifie dans l'ordre :
#   1. le Wi-Fi (pertes, latence) ;
#   2. l'écart entre l'horloge du portable et celle du robot : Nav2 et la TF
#      comparent les heures des messages du robot à celle du portable ;
#   3. ROS 2 : même domaine et même middleware des deux côtés, lidar et
#      odométrie du robot bien reçus sur le portable.
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
set +e

HOST="${ROBOT#*@}"
problems=0
warn() { echo "  ATTENTION : $*"; problems=$((problems + 1)); }

echo "== 1. Wi-Fi vers $HOST"
if ! command -v ping > /dev/null; then
    warn "ping absent du portable (sudo apt install iputils-ping) : Wi-Fi non mesuré"
    ping_out=""
else
    ping_out=$(ping -c 20 -i 0.2 -q "$HOST" 2>&1)
fi
if [[ -z "$ping_out" ]]; then
    :
elif [[ -z "$(echo "$ping_out" | grep 'packet loss')" ]]; then
    warn "pas de réponse au ping ($ping_out)"
else
    loss=$(echo "$ping_out" | grep -oE '[0-9.]+% packet loss' | cut -d% -f1)
    rtt=$(echo "$ping_out" | grep -oE '= [0-9.]+/[0-9.]+/[0-9.]+' | tr -d '= ')
    avg=$(echo "$rtt" | cut -d/ -f2); max=$(echo "$rtt" | cut -d/ -f3)
    echo "  pertes ${loss:-?} %, latence moyenne ${avg:-?} ms, maximale ${max:-?} ms"
    awk "BEGIN{exit !(${loss:-100} > 2)}" && warn "plus de 2 % de pertes : rapprochez-vous de la borne"
    awk "BEGIN{exit !(${max:-1000} > 100)}" && warn "pointes à plus de 100 ms : le robot réagira en retard"
fi

echo "== 2. Horloges"
best_rtt=""; best_off=""
for _ in 1 2 3 4 5; do
    t0=$(date +%s.%N); r=$(robot_ssh date +%s.%N 2>/dev/null); t1=$(date +%s.%N)
    [[ -z "$r" ]] && continue
    read -r rtt off < <(awk -v a="$t0" -v b="$t1" -v r="$r" \
        'BEGIN{printf "%.4f %.4f\n", b - a, r - (a + b) / 2}')
    if [[ -z "$best_rtt" ]] || awk "BEGIN{exit !($rtt < $best_rtt)}"; then
        best_rtt=$rtt; best_off=$off
    fi
done
if [[ -z "$best_off" ]]; then
    warn "impossible de lire l'heure du robot par SSH"
else
    echo "  robot - portable = $(awk -v o="$best_off" 'BEGIN{printf "%+.0f", o * 1000}') ms"
    if awk "BEGIN{o = $best_off; if (o < 0) o = -o; exit !(o > 0.05)}"; then
        warn "horloges décalées de plus de 50 ms. Calez le portable sur le robot :"
        echo "      sudo date -s \"@\$(ssh -p $ROBOT_PORT $ROBOT date +%s.%N)\""
        echo "    (ou demandez aux organisateurs le serveur NTP du robot)"
    fi
fi

echo "== 3. ROS 2"
remote_env=$(robot_ssh "bash -ic 'echo DOMAIN=\${ROS_DOMAIN_ID:-0} RMW=\${RMW_IMPLEMENTATION:-defaut}' 2>/dev/null" \
             | grep '^DOMAIN=' | tail -1)
local_env="DOMAIN=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-defaut}"
echo "  robot    : ${remote_env:-?}"
echo "  portable : $local_env"
[[ -n "$remote_env" && "$remote_env" != "$local_env" ]] && \
    warn "domaine ou middleware différents : exportez les mêmes valeurs sur le portable"
if ! command -v ros2 > /dev/null; then
    warn "ros2 introuvable sur le portable : sourcez ROS 2 Jazzy et votre espace de travail"
else
    for topic in /scan /botwheel_explorer/odom; do
        rate=$(timeout 8 ros2 topic hz "$topic" 2>/dev/null | grep -m1 'average rate' | awk '{print $3}')
        if [[ -z "$rate" ]]; then
            warn "$topic non reçu sur le portable. Essayez : export ROS_STATIC_PEERS=$HOST"
        else
            echo "  $topic reçu à $rate Hz"
        fi
    done
fi

echo
if [[ $problems -eq 0 ]]; then
    echo "Liaison prête : la solution peut tourner sur le portable (robot:=real_remote)."
else
    echo "$problems point(s) à corriger avant de faire rouler le robot à distance."
    exit 1
fi
