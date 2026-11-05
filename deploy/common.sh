# Fonctions communes aux scripts de deploy/ (à sourcer, ne pas exécuter).
#
# Principe : le compte du robot est partagé entre toutes les équipes. Un même
# utilisateur lit tous les fichiers de ce compte, donc les droits ne protègent
# rien. On n'y laisse rien :
#   - le code part du portable (rsync), jamais de git, de jeton ni de clé
#     privée sur le robot ;
#   - tout vit dans $ROBOT_DIR (notre dossier d'équipe, en mémoire vive par
#     défaut), y compris les journaux ROS, les fichiers temporaires, les paquets
#     Python et le modèle Whisper ;
#   - en fin de créneau, clean.sh récupère les cartes puis efface tout.

set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$DEPLOY_DIR")"
# Ce que le robot nous renvoie (cartes, enregistrements) : sur le portable.
LOCAL_DATA="${FOSA_LOCAL_DATA:-$REPO_DIR/robot_data}"

ENV_FILE="${FOSA_ROBOT_ENV:-$DEPLOY_DIR/robot.env}"
if [[ ! -f "$ENV_FILE" ]]; then
    echo "Fichier $ENV_FILE absent : copiez deploy/robot.env.example en deploy/robot.env." >&2
    exit 1
fi
# shellcheck disable=SC1090
source "$ENV_FILE"
: "${ROBOT:?ROBOT non défini dans $ENV_FILE}"
# Espace de travail de base des organisateurs (parc_msgs, pilotes du robot).
ROBOT_BASE_SETUP="${ROBOT_BASE_SETUP:-~/ros2_ws/install/setup.bash}"
ROBOT_PORT="${ROBOT_PORT:-22}"
ROBOT_DIR="${ROBOT_DIR:-/dev/shm/fosa}"

# Une seule connexion SSH par créneau : le mot de passe n'est demandé qu'une
# fois. Jamais de transfert d'agent (-A) vers un compte partagé.
SSH_OPTS=(-p "$ROBOT_PORT" -o ForwardAgent=no -o ControlMaster=auto
          -o "ControlPath=$HOME/.ssh/fosa-%r@%h:%p" -o ControlPersist=15m)
mkdir -p "$HOME/.ssh"

robot_ssh() {
    ssh "${SSH_OPTS[@]}" "$ROBOT" "$@"
}

robot_rsync() {
    rsync -a -e "ssh ${SSH_OPTS[*]}" "$@"
}

# Un « ~ » sans apostrophes dans robot.env a déjà été remplacé par le dossier
# personnel du portable : chemin du portable, sûrement faux sur le robot.
for variable in ROBOT_DIR ROBOT_BASE_SETUP; do
    if [[ "${!variable}" == "$HOME"/* ]]; then
        echo "ATTENTION : $variable=${!variable} est un chemin du portable." \
             "Écrire $variable='~/...' (avec apostrophes) dans $ENV_FILE." >&2
    fi
done
# « ~/... » dans robot.env désigne le dossier personnel DU ROBOT : on le remplace
# par le chemin absolu (une connexion, qui ouvre aussi la connexion partagée).
if [[ "$ROBOT_DIR$ROBOT_BASE_SETUP" == *"~"* ]]; then
    ROBOT_HOME="$(robot_ssh 'printf %s "$HOME"')" \
        || { echo "Connexion SSH à $ROBOT impossible." >&2; exit 1; }
    ROBOT_DIR="${ROBOT_DIR/#\~/$ROBOT_HOME}"
    ROBOT_BASE_SETUP="${ROBOT_BASE_SETUP/#\~/$ROBOT_HOME}"
fi
case "$ROBOT_DIR" in
    /dev/shm/*|/run/user/*|/tmp/*) ;;
    /|"$HOME"|"${ROBOT_HOME:-/nonexistent}"|/home|/home/*/)
        echo "ROBOT_DIR=$ROBOT_DIR : il faut un sous-dossier à nous (clean.sh l'efface entier)." >&2
        exit 1 ;;
    *) echo "Note : $ROBOT_DIR est sur disque (dossier d'équipe) : il survit au redémarrage," \
            "deploy/clean.sh l'efface en fin de créneau." >&2 ;;
esac
