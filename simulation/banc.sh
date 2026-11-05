#!/usr/bin/env bash
# Banc de simulation Fosa : rejoue toute la solution sans le robot et juge le
# résultat sur la vérité terrain. Mode d'emploi : simulation/README.md.
#
#   simulation/banc.sh 2d tout                     tout, dans le simulateur 2D (~45 min)
#   simulation/banc.sh 2d carte|reperage|course|personne|tag_absent|reel
#   simulation/banc.sh 2d arbitre [--depart commande|voix]   jugée par parc_referee
#   simulation/banc.sh gazebo course --carte ~/fosa_maps/X [--personne] [--sans-tag 9]
#   simulation/banc.sh voix [dossier_wav]
#   simulation/banc.sh nettoyer                    arrête ce qu'un banc interrompu a laissé
#
# Options : --ordre 7,1,9,4|aleatoire   --type-ordre texte|entiers|octets
#           --carte DIR (2d : carte à utiliser au lieu de celle du banc)  --depart commande|voix
set -uo pipefail

SIMU="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPOT="$(dirname "$SIMU")"
RESULTATS="${FOSA_SIM_RESULTATS:-$SIMU/resultats}"
CARTE_2D="$RESULTATS/cartes/sim2d"
GROUPES="$RESULTATS/.groupes"
mkdir -p "$RESULTATS"

usage() { sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

# ------------------------------------------------------------------ arguments
SIMULATEUR="${1:-}"; [[ -n "$SIMULATEUR" ]] || usage 1
shift
SCENARIO=""
if [[ "$SIMULATEUR" == 2d || "$SIMULATEUR" == gazebo ]]; then
    SCENARIO="${1:-}"; [[ -n "$SCENARIO" ]] || usage 1
    shift
fi
ORDRE="7,1,9,4"; TYPE_ORDRE="texte"; CARTE=""; PERSONNE=0; SANS_TAG=""; DOSSIER_VOIX=""
DEPART="commande"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ordre) ORDRE="$2"; shift 2 ;;
        --type-ordre) TYPE_ORDRE="$2"; shift 2 ;;
        --carte) CARTE="$(cd "$2" && pwd)"; shift 2 ;;
        --personne) PERSONNE=1; shift ;;
        --sans-tag) SANS_TAG="$2"; shift 2 ;;
        --depart) DEPART="$2"; shift 2 ;;
        -h|--help) usage 0 ;;
        *) if [[ "$SIMULATEUR" == voix && -z "$DOSSIER_VOIX" ]]; then DOSSIER_VOIX="$1"; shift
           else echo "Option inconnue : $1" >&2; usage 1; fi ;;
    esac
done

# ------------------------------------------------------------- processus
# Chaque programme part dans son propre groupe (setsid) : on l'arrête par son
# groupe, jamais par un motif de nom (qui pourrait tuer autre chose).
lancer() {   # nom, commande... ; journal dans $DOSSIER/<nom>.log
    local nom="$1"; shift
    setsid "$@" > "$DOSSIER/$nom.log" 2>&1 < /dev/null &
    local pid=$!
    echo "$pid" >> "$GROUPES"
    eval "PID_${nom}=$pid"
}

arreter_groupe() {   # pid
    local pid="$1" i
    kill -INT -- "-$pid" 2> /dev/null || return 0
    for i in $(seq 1 20); do kill -0 -- "-$pid" 2> /dev/null || return 0; sleep 0.5; done
    kill -TERM -- "-$pid" 2> /dev/null; sleep 2
    kill -KILL -- "-$pid" 2> /dev/null || true
}

arreter_tout() {
    [[ -f "$GROUPES" ]] || return 0
    local pid
    # Ordre inverse : la course avant le simulateur (vitesse nulle d'abord).
    for pid in $(tac "$GROUPES"); do arreter_groupe "$pid"; done
    rm -f "$GROUPES"
}
trap 'arreter_tout' EXIT
trap 'echo "Interrompu."; exit 130' INT TERM

attendre_ligne() {   # fichier, motif (grep -E), délai en s
    local i
    for i in $(seq 1 "$3"); do
        grep -qE "$2" "$1" 2> /dev/null && return 0
        sleep 1
    done
    return 1
}

noter() {   # ligne du rapport global
    echo "$*" | tee -a "$RAPPORT"
}

verifier_environnement() {
    command -v ros2 > /dev/null || { echo "ROS 2 non chargé : source /opt/ros/jazzy/setup.bash" >&2; exit 1; }
    ros2 pkg prefix caytu_nav_bringup > /dev/null 2>&1 \
        || { echo "Workspace non chargé : source <ws>/install/setup.bash (colcon build fait ?)" >&2; exit 1; }
    python3 -c "import cv2, numpy, yaml; cv2.aruco" 2> /dev/null \
        || { echo "python3 : numpy, yaml et OpenCV avec aruco sont nécessaires." >&2; exit 1; }
}

nouveau_dossier() {   # nom du scénario
    DOSSIER="$SESSION/$1"
    rm -rf "$DOSSIER"; mkdir -p "$DOSSIER"
    rm -f "$GROUPES"
}

# ---------------------------------------------------------------- 2D
environnement_2d() {   # sim | reel
    # Domaine à part : rien ne se mélange avec un robot ou un Gazebo ouvert.
    export ROS_DOMAIN_ID="${FOSA_SIM_DOMAIN:-77}"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    if [[ "$1" == reel ]]; then
        local prefixe
        prefixe="$(ros2 pkg prefix caytu_config)"
        export FOSA_CONFIG_OVERRIDE="$prefixe/share/caytu_config/config/robot_reel.yaml"
    else
        unset FOSA_CONFIG_OVERRIDE
    fi
}

scenario_carte_2d() {
    nouveau_dossier carte
    environnement_2d sim
    rm -rf "$CARTE_2D"; mkdir -p "$(dirname "$CARTE_2D")"
    lancer simulateur python3 "$SIMU/sim2d/simulateur.py"
    sleep 3
    lancer cartographie ros2 launch caytu_nav_bringup mapping.launch.py gazebo:=true \
        map_name:="$(basename "$CARTE_2D")" maps_dir:="$(dirname "$CARTE_2D")" \
        task_params_file:="$SIMU/sim2d/task_params.yaml"
    sleep 12
    timeout 420 python3 "$SIMU/sim2d/conduite_auto.py" > "$DOSSIER/conduite.log" 2>&1
    sleep 3
    ros2 run caytu_nav_solution save_map > "$DOSSIER/save_map.log" 2>&1
    sleep 3
    arreter_tout
    local tags
    tags=$(python3 -c "import yaml,sys; print(len((yaml.safe_load(open(sys.argv[1])) or {}).get('tags') or {}))" \
           "$CARTE_2D/tags.yaml" 2> /dev/null || echo 0)
    if [[ -f "$CARTE_2D/fosa_map.yaml" && "$tags" -ge 4 ]]; then
        noter "[carte]      RÉUSSI  carte $CARTE_2D, $tags tags notés"
    else
        noter "[carte]      ÉCHEC   carte ou tags manquants ($tags tags) : voir $DOSSIER"
    fi
    ecarts_tags_2d >> "$RAPPORT"
}

ecarts_tags_2d() {   # écart entre tags.yaml (repère map) et la vérité du simulateur
    python3 - "$CARTE_2D" "$SIMU/sim2d/simulateur.py" <<'PYTHON'
import math, sys, yaml
carte, source = sys.argv[1], open(sys.argv[2], encoding='utf-8').read()
debut = source.index('TAGS = {')
espace = {'math': math}
exec(source[debut:source.index('}', debut) + 1], espace)
meta = yaml.safe_load(open(f'{carte}/fosa_map.yaml'))
notes = (yaml.safe_load(open(f'{carte}/tags.yaml')) or {}).get('tags') or {}
wx, wy, wl = meta['world_from_map']
for i, (x, y, cap) in sorted(espace['TAGS'].items()):
    if i not in notes:
        print(f'               tag {i} : jamais noté')
        continue
    n = notes[i]
    mx = wx + math.cos(wl) * n['x'] - math.sin(wl) * n['y']
    my = wy + math.sin(wl) * n['x'] + math.cos(wl) * n['y']
    print(f'               tag {i} : noté à {math.hypot(mx - x, my - y):.2f} m de sa vraie place')
PYTHON
}

scenario_reperage_2d() {
    nouveau_dossier reperage
    environnement_2d sim
    [[ -f "$CARTE_2D/fosa_map.yaml" ]] || { noter "[reperage]   ÉCHEC   pas de carte : lancer d'abord « 2d carte »"; return; }
    lancer simulateur python3 "$SIMU/sim2d/simulateur.py"
    sleep 3
    lancer reperage ros2 launch caytu_nav_bringup reperage_tags.launch.py robot:=sim carte:="$CARTE_2D"
    attendre_ligne "$DOSSIER/reperage.log" "Managed nodes are active|Navigation prête" 120
    sleep 5
    timeout 420 python3 "$SIMU/sim2d/conduite_auto.py" > "$DOSSIER/conduite.log" 2>&1
    sleep 4
    arreter_tout
    local reperes
    reperes=$(grep -c "repéré" "$DOSSIER/reperage.log" 2> /dev/null || echo 0)
    noter "[reperage]   $( [[ $reperes -ge 4 ]] && echo 'RÉUSSI ' || echo 'ÉCHEC  ') $reperes annonces « Tag repéré »"
    ecarts_tags_2d >> "$RAPPORT"
}

course_2d() {   # nom, mode (sim|reel), options du simulateur...
    local nom="$1" mode="$2"; shift 2
    nouveau_dossier "$nom"
    environnement_2d "$mode"
    local carte="${CARTE:-$CARTE_2D}"
    [[ -f "$carte/fosa_map.yaml" ]] || { noter "[$nom] ÉCHEC   pas de carte : lancer d'abord « 2d carte »"; return; }
    local robot=sim options_sim=() visites_min=4
    [[ "$mode" == reel ]] && robot=real && options_sim=(--reel)
    [[ " $* " == *" --sans-tag "* ]] && visites_min=3
    lancer simulateur python3 "$SIMU/sim2d/simulateur.py" "${options_sim[@]}" "$@"
    sleep 3
    lancer arbitre python3 "$SIMU/outils/verifier_course.py" --sortie "$DOSSIER/resultat.json" \
        --visites-min "$visites_min"
    lancer ordre python3 "$SIMU/outils/ordre_organisateurs.py" "$ORDRE" --type "$TYPE_ORDRE"
    lancer course ros2 launch caytu_nav_bringup course.launch.py robot:="$robot" \
        carte:="$carte" voix:=false
    if ! attendre_ligne "$DOSSIER/course.log" "Prêt. Départ" 180; then
        noter "[$nom] ÉCHEC   la course n'est jamais prête (voir $DOSSIER/course.log)"
        arreter_tout; return
    fi
    sleep 10                                 # le robot doit rester immobile
    ros2 run caytu_nav_solution depart > "$DOSSIER/depart.log" 2>&1
    local i
    for i in $(seq 1 720); do kill -0 "$PID_arbitre" 2> /dev/null || break; sleep 1; done
    sleep 2
    arreter_tout
    bilan_course "$nom" "$DOSSIER/simulateur.log"
}

bilan_course() {   # nom, journal du simulateur (contacts) ou vide
    local nom="$1" journal="${2:-}" contacts=0 verdict
    [[ -n "$journal" ]] && contacts=$(grep -c "CONTACT" "$journal" 2> /dev/null)
    contacts=${contacts:-0}
    verdict=$(python3 - "$DOSSIER/resultat.json" "$contacts" <<'PYTHON'
import json, sys
try:
    r = json.load(open(sys.argv[1]))
except Exception:
    print('ÉCHEC   pas de résultat de l\'arbitre'); sys.exit()
contacts = int(sys.argv[2])
ok = r['reussi'] and contacts == 0
details = [f"tags valides {r['tags_valides']} (ordre {r['ordre']})",
           f"fin {round(r['fin']['temps_s']) if r.get('fin') else 'absente'} s",
           f"immobile avant départ {'oui' if r['immobile_avant_depart'] else 'NON'}",
           f"contacts {contacts}"]
if r.get('distance_min_personne_m') is not None:
    details.append(f"personne à {r['distance_min_personne_m']} m au plus près")
print(('RÉUSSI ' if ok else 'ÉCHEC  ') + ' ; '.join(details))
PYTHON
)
    noter "$(printf '[%-10s] %s' "$nom" "$verdict")"
    grep -E "^VISITE|^SAUTE" "$DOSSIER/arbitre.log" 2> /dev/null | sed 's/^/               /' >> "$RAPPORT"
}

# ------------------------------------------------- arbitre officiel (2D)
# Course aux interfaces du vrai robot, jugée par l'arbitre des organisateurs
# (parc_referee du dépôt PARC2026-Engineers-League) : ordre sur /parc/tag_order,
# départ, visites avec image et fin sur /parc/*, faux départ et pause de 3 s
# contrôlés sur l'odométrie. Notre vérificateur (vérité terrain) tourne aussi.
scenario_arbitre_2d() {
    nouveau_dossier arbitre
    environnement_2d reel
    if ! ros2 pkg prefix parc_referee > /dev/null 2>&1; then
        noter "[arbitre   ] ÉCHEC   parc_referee absent : compiler le dépôt PARC2026-Engineers-League dans le workspace"
        return
    fi
    local carte="${CARTE:-$CARTE_2D}" port="${FOSA_ARBITRE_PORT:-8080}"
    [[ -f "$carte/fosa_map.yaml" ]] || { noter "[arbitre   ] ÉCHEC   pas de carte : lancer d'abord « 2d carte » et « 2d reperage »"; return; }
    lancer simulateur python3 "$SIMU/sim2d/simulateur.py" --reel
    sleep 3
    lancer arbitre_officiel ros2 run parc_referee referee --ros-args \
        -p odom_topic:=/botwheel_explorer/odom -p web_port:="$port" \
        -p log_dir:="$DOSSIER/parc_runs"
    lancer arbitre python3 "$SIMU/outils/verifier_course.py" --sortie "$DOSSIER/resultat.json" \
        --visites-min 4
    if ! python3 "$SIMU/outils/arbitre_officiel.py" armer --ordre "$ORDRE" --port "$port" \
            > "$DOSSIER/armement.log" 2>&1; then
        noter "[arbitre   ] ÉCHEC   $(tail -1 "$DOSSIER/armement.log")"; arreter_tout; return
    fi
    lancer course ros2 launch caytu_nav_bringup course.launch.py robot:=real \
        carte:="$carte" voix:=false
    if ! attendre_ligne "$DOSSIER/course.log" "Prêt. Départ" 180; then
        noter "[arbitre   ] ÉCHEC   la course n'est jamais prête (voir $DOSSIER/course.log)"
        arreter_tout; return
    fi
    sleep 10                                 # immobile : l'arbitre guette le faux départ
    if [[ "$DEPART" == voix ]]; then
        # Ce que publie mot_depart quand Whisper reconnaît le mot (banc « voix » à part).
        ros2 topic pub --once -w 1 /fosa/depart_vocal std_msgs/msg/String "{data: allez}" \
            > "$DOSSIER/depart.log" 2>&1
    else
        ros2 run caytu_nav_solution depart > "$DOSSIER/depart.log" 2>&1
    fi
    python3 "$SIMU/outils/arbitre_officiel.py" attendre --port "$port" \
        --sortie "$DOSSIER/arbitre_officiel.json" > "$DOSSIER/bilan_officiel.log" 2>&1
    local i
    for i in $(seq 1 30); do kill -0 "$PID_arbitre" 2> /dev/null || break; sleep 1; done
    arreter_tout
    bilan_course arbitre "$DOSSIER/simulateur.log"
    noter "             arbitre officiel : $(head -1 "$DOSSIER/bilan_officiel.log")"
    tail -n +2 "$DOSSIER/bilan_officiel.log" | sed 's/^/           /' >> "$RAPPORT"
    grep -q "^RÉUSSI" "$DOSSIER/bilan_officiel.log" || noter "[arbitre   ] ÉCHEC   selon l'arbitre officiel"
}

# ------------------------------------------------------------- Gazebo
scenario_course_gazebo() {
    nouveau_dossier gazebo_course
    [[ -n "$CARTE" && -f "$CARTE/fosa_map.yaml" ]] || { echo "--carte <dossier de la carte faite dans Gazebo> requis" >&2; exit 1; }
    command -v gz > /dev/null || { echo "Commande gz introuvable (Gazebo Harmonic)." >&2; exit 1; }
    python3 "$DEPOT/tools/tags_fictifs.py" "$CARTE" > "$DOSSIER/tags_fictifs.log" 2>&1 \
        || { cat "$DOSSIER/tags_fictifs.log"; exit 1; }
    local visites_min=4
    if [[ -n "$SANS_TAG" ]]; then
        python3 "$DEPOT/tools/tags_fictifs.py" "$CARTE" --retirer --ids "$SANS_TAG" >> "$DOSSIER/tags_fictifs.log" 2>&1
        visites_min=3
    fi
    lancer arbitre python3 "$SIMU/outils/verifier_course.py" --sortie "$DOSSIER/resultat.json" \
        --verite "$CARTE/tags_fictifs/verite.yaml" --visites-min "$visites_min"
    lancer ordre python3 "$SIMU/outils/ordre_organisateurs.py" "$ORDRE" --type "$TYPE_ORDRE"
    [[ $PERSONNE == 1 ]] && lancer personne python3 "$SIMU/outils/obstacle_gazebo.py" "$CARTE"
    lancer course ros2 launch caytu_nav_bringup course.launch.py robot:=sim carte:="$CARTE" voix:=false
    if ! attendre_ligne "$DOSSIER/course.log" "Prêt. Départ" 240; then
        noter "[gazebo]     ÉCHEC   la course n'est jamais prête (voir $DOSSIER/course.log)"
        arreter_tout; return
    fi
    sleep 10
    ros2 run caytu_nav_solution depart > "$DOSSIER/depart.log" 2>&1
    local i
    for i in $(seq 1 720); do kill -0 "$PID_arbitre" 2> /dev/null || break; sleep 1; done
    sleep 2
    arreter_tout
    bilan_course gazebo ""
    echo "Les tags fictifs restent dans Gazebo ; relancer Gazebo avant une autre course (robot au départ)."
}

# --------------------------------------------------------------- voix
scenario_voix() {
    local dossier="${DOSSIER_VOIX:-$SIMU/audios}"
    if ! ls "$dossier"/oui_*.wav > /dev/null 2>&1; then
        python3 "$SIMU/outils/generer_audios.py" "$dossier"
    fi
    if python3 "$SIMU/outils/test_voix.py" "$dossier" | tee "$SESSION/voix.log"; then
        noter "[voix]       RÉUSSI  $(tail -1 "$SESSION/voix.log")"
    else
        noter "[voix]       ÉCHEC   $(tail -1 "$SESSION/voix.log")"
    fi
}

# --------------------------------------------------------------- principal
if [[ "$SIMULATEUR" == nettoyer ]]; then
    DOSSIER="$RESULTATS"; arreter_tout; echo "Programmes du banc arrêtés."; exit 0
fi
verifier_environnement
SESSION="$RESULTATS/$(date +%Y%m%d_%H%M%S)_${SIMULATEUR}${SCENARIO:+_$SCENARIO}"
mkdir -p "$SESSION"
RAPPORT="$SESSION/rapport.txt"
echo "Banc Fosa $SIMULATEUR $SCENARIO — $(date '+%d/%m/%Y %H:%M') — ordre $ORDRE ($TYPE_ORDRE)" > "$RAPPORT"
echo "Journaux et résultats : $SESSION"

case "$SIMULATEUR:$SCENARIO" in
    2d:carte) scenario_carte_2d ;;
    2d:reperage) scenario_reperage_2d ;;
    2d:course) course_2d course sim ;;
    2d:personne) course_2d personne sim --personne ;;
    2d:tag_absent) course_2d tag_absent sim --sans-tag "${SANS_TAG:-9}" ;;
    2d:reel) course_2d reel reel ;;
    2d:arbitre) scenario_arbitre_2d ;;
    2d:tout)
        scenario_carte_2d
        scenario_reperage_2d
        course_2d course sim
        course_2d personne sim --personne
        course_2d tag_absent sim --sans-tag "${SANS_TAG:-9}"
        course_2d reel reel
        ;;
    gazebo:course) scenario_course_gazebo ;;
    voix:) scenario_voix ;;
    *) echo "Scénario inconnu : $SIMULATEUR $SCENARIO" >&2; usage 1 ;;
esac
echo
echo "================ RAPPORT ================"
cat "$RAPPORT"
grep -q "ÉCHEC" "$RAPPORT" && exit 1 || exit 0
