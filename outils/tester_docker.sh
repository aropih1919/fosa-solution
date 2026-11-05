#!/usr/bin/env bash
# À lancer depuis n'importe quel dossier ; seul results/ reçoit des écritures.
set -euo pipefail

racine="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image=fosa-tests:jazzy
if [[ $# -ne 0 ]]; then
    echo 'Usage : bash outils/tester_docker.sh' >&2
    exit 2
fi
docker info > /dev/null
mkdir -p "$racine/results/docker"
rapports="$(mktemp -d "$racine/results/docker/essai-XXXXXXXX")"
echo "Rapports : $rapports"
git -C "$racine" rev-parse HEAD > "$rapports/commit.txt"
git -C "$racine" status --short > "$rapports/etat-git.txt"

docker build --file "$racine/docker/tests/Dockerfile" --tag "$image" "$racine" \
    2>&1 | tee "$rapports/image.log"
docker image inspect --format '{{.Id}}' "$image" > "$rapports/image-id.txt"

# Réseau et périphériques du robot inaccessibles. UID/GID de l'utilisateur :
# les rapports restent lisibles/modifiables sans sudo sur le poste Linux.
docker run --rm --network none --cap-drop ALL \
    --security-opt no-new-privileges \
    --user "$(id -u):$(id -g)" \
    --mount "type=bind,source=$racine,target=/depot,readonly" \
    --mount "type=bind,source=$rapports,target=/rapports" \
    "$image" 2>&1 | tee "$rapports/tests.log"
