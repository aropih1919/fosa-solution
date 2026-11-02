#!/usr/bin/env bash
# Exporte ou importe les modèles locaux utilisés par la solution Fosa.
set -euo pipefail

DOSSIER_PROJET="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ARCHIVE_MODELES="$DOSSIER_PROJET/modeles-fosa.tar.gz"
DOSSIER_CONFIG="$DOSSIER_PROJET/caytu_config/config"
DOSSIER_MODELES_CONFIG="$DOSSIER_CONFIG/models"
CACHE_HUGGINGFACE="${HF_HUB_CACHE:-${HUGGINGFACE_HUB_CACHE:-${HF_HOME:-$HOME/.cache/huggingface}/hub}}"
DOSSIER_TEMPORAIRE=""
SERVICE_OLLAMA_ACTIF="non"

afficher_aide() {
    cat <<'AIDE'
Usage : ./exporter_importer_modeles.sh exporter|importer

  exporter   crée modeles-fosa.tar.gz à la racine du projet
  importer   installe les modèles depuis modeles-fosa.tar.gz

Sur le PC destinataire, installer d'abord le projet sans télécharger les
modèles avec ./install.sh --no-models, copier l'archive à la racine du projet,
puis lancer ./exporter_importer_modeles.sh importer.
AIDE
}

nettoyer() {
    if [[ -n "$DOSSIER_TEMPORAIRE" && -d "$DOSSIER_TEMPORAIRE" ]]; then
        rm -rf -- "$DOSSIER_TEMPORAIRE"
    fi
    if [[ "$SERVICE_OLLAMA_ACTIF" == "oui" ]]; then
        sudo systemctl start ollama
    fi
}
trap nettoyer EXIT

erreur() {
    echo "Erreur : $*" >&2
    exit 1
}

obtenir_configuration_ollama() {
    UTILISATEUR_OLLAMA="$(systemctl show ollama --property=User --value 2>/dev/null || true)"
    GROUPE_OLLAMA="$(systemctl show ollama --property=Group --value 2>/dev/null || true)"
    MODELES_OLLAMA_SERVICE="$(systemctl show ollama --property=Environment --value 2>/dev/null \
        | python3 -c 'import shlex,sys; print(next((v.split("=",1)[1] for v in shlex.split(sys.stdin.read()) if v.startswith("OLLAMA_MODELS=")), ""))' \
        2>/dev/null || true)"

    if [[ -n "$MODELES_OLLAMA_SERVICE" ]]; then
        DOSSIER_MODELES_OLLAMA="$MODELES_OLLAMA_SERVICE"
    elif [[ -n "${OLLAMA_MODELS:-}" ]]; then
        DOSSIER_MODELES_OLLAMA="$OLLAMA_MODELS"
    elif [[ -n "$UTILISATEUR_OLLAMA" ]]; then
        if [[ "$UTILISATEUR_OLLAMA" == "ollama" ]]; then
            DOSSIER_MODELES_OLLAMA="/usr/share/ollama/.ollama/models"
        else
            DOSSIER_OLLAMA_UTILISATEUR="$(getent passwd "$UTILISATEUR_OLLAMA" | cut -d: -f6)"
            [[ -n "$DOSSIER_OLLAMA_UTILISATEUR" ]] || erreur \
                "impossible de trouver le dossier personnel du compte Ollama '$UTILISATEUR_OLLAMA'."
            DOSSIER_MODELES_OLLAMA="$DOSSIER_OLLAMA_UTILISATEUR/.ollama/models"
        fi
    elif [[ -d "$HOME/.ollama/models" ]]; then
        UTILISATEUR_OLLAMA="$(id -un)"
        GROUPE_OLLAMA="$(id -gn)"
        DOSSIER_MODELES_OLLAMA="$HOME/.ollama/models"
    else
        erreur "répertoire de modèles Ollama introuvable. Définissez OLLAMA_MODELS ou démarrez Ollama."
    fi

    if [[ -z "$UTILISATEUR_OLLAMA" ]]; then
        UTILISATEUR_OLLAMA="$(id -un)"
    fi
    if [[ -z "$GROUPE_OLLAMA" ]]; then
        GROUPE_OLLAMA="$(id -gn "$UTILISATEUR_OLLAMA")"
    fi
}

obtenir_depot_whisper() {
    MODELE_WHISPER="$(python3 - "$DOSSIER_CONFIG/fosa.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as fichier:
    print(yaml.safe_load(fichier)["dialogue"]["stt"]["local_model"])
PY
)"
    if [[ "$MODELE_WHISPER" == */* ]]; then
        DEPOT_WHISPER="$MODELE_WHISPER"
    else
        DEPOT_WHISPER="Systran/faster-whisper-$MODELE_WHISPER"
    fi
    NOM_DEPOT_WHISPER="models--${DEPOT_WHISPER//\//--}"
    DOSSIER_CACHE_WHISPER="$CACHE_HUGGINGFACE/$NOM_DEPOT_WHISPER"
}

obtenir_modeles_configures() {
    readarray -t CHEMINS_MODELES < <(python3 - "$DOSSIER_CONFIG/fosa.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as fichier:
    configuration = yaml.safe_load(fichier)
print(configuration["perception"]["yolo"]["model_path"])
print(configuration["dialogue"]["tts"]["local_voice"])
print(configuration["dialogue"]["llm"]["local_model"])
PY
)
    CHEMIN_YOLO="${CHEMINS_MODELES[0]}"
    CHEMIN_VOIX="${CHEMINS_MODELES[1]}"
    MODELE_OLLAMA="${CHEMINS_MODELES[2]}"
    [[ "$CHEMIN_YOLO" != /* && "$CHEMIN_VOIX" != /* ]] || erreur \
        "les chemins YOLO et Piper doivent être relatifs à la configuration pour être transférables."
    FICHIER_YOLO="$DOSSIER_CONFIG/$CHEMIN_YOLO"
    FICHIER_VOIX="$DOSSIER_CONFIG/$CHEMIN_VOIX"
}

exporter_importer_modeles() {
    [[ -f "$DOSSIER_CONFIG/fosa.yaml" ]] || erreur \
        "configuration du projet absente : $DOSSIER_CONFIG/fosa.yaml"
    [[ -d "$DOSSIER_MODELES_CONFIG" ]] || erreur \
        "dossier de modèles absent : $DOSSIER_MODELES_CONFIG"

    obtenir_depot_whisper
    obtenir_modeles_configures
    [[ -d "$DOSSIER_CACHE_WHISPER" ]] || erreur \
        "cache Whisper absent : $DOSSIER_CACHE_WHISPER. Lancez ./install.sh --models."

    obtenir_configuration_ollama
    [[ -d "$DOSSIER_MODELES_OLLAMA" ]] || erreur \
        "dossier Ollama absent : $DOSSIER_MODELES_OLLAMA"
    ollama list | awk -v modele="$MODELE_OLLAMA" 'NR > 1 && $1 == modele { trouve = 1 } END { exit !trouve }' \
        || erreur "modèle Ollama '$MODELE_OLLAMA' absent. Lancez ollama pull $MODELE_OLLAMA."

    [[ -f "$FICHIER_YOLO" ]] || erreur \
        "modèle YOLO absent : $FICHIER_YOLO. Lancez ./install.sh --models."
    [[ -f "$FICHIER_VOIX" && -f "$FICHIER_VOIX.json" ]] || erreur \
        "modèle ou métadonnées Piper absents pour '$FICHIER_VOIX'. Lancez ./install.sh --models."

    DOSSIER_TEMPORAIRE="$(mktemp -d "${TMPDIR:-/tmp}/fosa-modeles-export.XXXXXX")"
    mkdir -p "$DOSSIER_TEMPORAIRE/modeles/config" \
        "$DOSSIER_TEMPORAIRE/modeles/huggingface/hub" \
        "$DOSSIER_TEMPORAIRE/modeles/ollama/models"
    cp -a "$DOSSIER_MODELES_CONFIG" "$DOSSIER_TEMPORAIRE/modeles/config/"
    cp -a "$DOSSIER_CACHE_WHISPER" \
        "$DOSSIER_TEMPORAIRE/modeles/huggingface/hub/"
    sudo cp -a "$DOSSIER_MODELES_OLLAMA/." \
        "$DOSSIER_TEMPORAIRE/modeles/ollama/models/"
    sudo chown -R "$(id -u):$(id -g)" \
        "$DOSSIER_TEMPORAIRE/modeles/ollama/models"

    tar -czf "$ARCHIVE_MODELES.tmp" -C "$DOSSIER_TEMPORAIRE" modeles
    mv -f -- "$ARCHIVE_MODELES.tmp" "$ARCHIVE_MODELES"
    echo "Archive créée : $ARCHIVE_MODELES"
    du -h "$ARCHIVE_MODELES"
}

importer_modeles() {
    [[ -f "$ARCHIVE_MODELES" ]] || erreur \
        "archive absente : $ARCHIVE_MODELES. Copiez-la à la racine du projet."
    [[ -f "$DOSSIER_CONFIG/fosa.yaml" ]] || erreur \
        "configuration du projet absente : $DOSSIER_CONFIG/fosa.yaml"
    obtenir_depot_whisper
    obtenir_modeles_configures

    DOSSIER_TEMPORAIRE="$(mktemp -d "${TMPDIR:-/tmp}/fosa-modeles-import.XXXXXX")"
    if tar -tzf "$ARCHIVE_MODELES" | awk '/(^\/|(^|\/)\.\.(\/|$))/ { invalide = 1 } END { exit invalide }'; then
        tar -xzf "$ARCHIVE_MODELES" -C "$DOSSIER_TEMPORAIRE"
    else
        erreur "l'archive contient un chemin invalide."
    fi
    RACINE_ARCHIVE="$DOSSIER_TEMPORAIRE/modeles"
    [[ -d "$RACINE_ARCHIVE/config/models" ]] || erreur \
        "l'archive ne contient pas les modèles YOLO/Piper attendus."
    [[ -d "$RACINE_ARCHIVE/huggingface/hub" ]] || erreur \
        "l'archive ne contient pas le cache Whisper attendu."
    [[ -d "$RACINE_ARCHIVE/ollama/models/manifests" \
        && -d "$RACINE_ARCHIVE/ollama/models/blobs" ]] || erreur \
        "l'archive ne contient pas les modèles Ollama attendus."

    mkdir -p "$DOSSIER_MODELES_CONFIG" "$CACHE_HUGGINGFACE"
    cp -a "$RACINE_ARCHIVE/config/models/." "$DOSSIER_MODELES_CONFIG/"
    cp -a "$RACINE_ARCHIVE/huggingface/hub/." "$CACHE_HUGGINGFACE/"

    obtenir_configuration_ollama
    if systemctl is-active --quiet ollama 2>/dev/null; then
        sudo systemctl stop ollama
        SERVICE_OLLAMA_ACTIF="oui"
    fi
    sudo mkdir -p "$DOSSIER_MODELES_OLLAMA"
    sudo cp -a "$RACINE_ARCHIVE/ollama/models/." "$DOSSIER_MODELES_OLLAMA/"
    sudo chown -R "$UTILISATEUR_OLLAMA:$GROUPE_OLLAMA" "$DOSSIER_MODELES_OLLAMA"
    if [[ "$SERVICE_OLLAMA_ACTIF" == "oui" ]]; then
        sudo systemctl start ollama
        SERVICE_OLLAMA_ACTIF="non"
    fi

    echo "Modèles installés dans :"
    echo "  YOLO/Piper : $DOSSIER_MODELES_CONFIG"
    echo "  Whisper    : $CACHE_HUGGINGFACE/$NOM_DEPOT_WHISPER"
    echo "  Ollama     : $DOSSIER_MODELES_OLLAMA"
    if ! ollama list; then
        echo "Démarrez Ollama pour vérifier les modèles avec : ollama list"
    fi
}

mode="${1:-}"
case "$mode" in
    exporter)
        (($# == 1)) || { afficher_aide >&2; exit 2; }
        exporter_importer_modeles
        ;;
    importer)
        (($# == 1)) || { afficher_aide >&2; exit 2; }
        importer_modeles
        ;;
    aide|--help|-h)
        afficher_aide
        ;;
    *)
        afficher_aide >&2
        exit 2
        ;;
esac
