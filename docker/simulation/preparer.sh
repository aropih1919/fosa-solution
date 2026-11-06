#!/usr/bin/env bash
# Validation Jazzy isolée : les sources officielles sont celles du workspace PC.
set -euo pipefail
depot="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
racine_ros="${FOSA_WS:-$(cd -- "$depot/../.." && pwd)}"
sources_parc="${FOSA_PARC_SRC:-$racine_ros/src}"
workspace="${FOSA_SIM_WORKSPACE:-$depot/results/validation-$(date +%Y%m%d_%H%M%S)}"
image="${FOSA_IMAGE_TESTS:-fosa-finale:jazzy}"
for paquet in parc_msgs parc_referee parc_robot_bringup parc_robot_description; do
    [[ -f "$sources_parc/$paquet/package.xml" ]] || {
        echo "Paquet officiel absent : $sources_parc/$paquet" >&2; exit 1;
    }
done
[[ ! -e "$workspace/src" ]] || { echo "Workspace déjà utilisé : $workspace" >&2; exit 1; }
mkdir -p "$workspace"
if [[ "${FOSA_IMAGE_PRETE:-0}" != 1 ]]; then
    docker build -f "$depot/docker/simulation/Dockerfile" -t "$image" "$depot/docker/simulation"
fi
docker run --rm --network none --shm-size=1g --user "$(id -u):$(id -g)" \
    --entrypoint bash -e HOME=/workspace/.maison -e FOSA_TEST_DDS_ISOLE=1 \
    -e FOSA_TEST_ROS_ISOLE=1 -e FOSA_SIM_GUI=0 -e LIBGL_ALWAYS_SOFTWARE=1 \
    --mount "type=bind,src=$depot,dst=/depot,readonly" \
    --mount "type=bind,src=$sources_parc,dst=/parc,readonly" \
    --mount "type=bind,src=$workspace,dst=/workspace" \
    -w /workspace "$image" -c '
set -eo pipefail
mkdir -p "$HOME" /workspace/src
python3 /depot/outils/livraison.py creer /workspace/fosa.zip
python3 -m zipfile -e /workspace/fosa.zip /workspace
python3 - <<"PYTHON"
from pathlib import Path
import zipfile
with zipfile.ZipFile("/workspace/fosa.zip") as archive:
    for info in archive.infolist():
        Path("/workspace", info.filename).chmod((info.external_attr >> 16) & 0o777)
PYTHON
for paquet in parc_msgs parc_referee parc_robot_bringup parc_robot_description; do
    cp -r "/parc/$paquet" /workspace/src/
done
source /opt/ros/jazzy/setup.bash
bash /workspace/src/fosa-solution/simulation/simulation.sh preparer
source /workspace/install/setup.bash
bash /workspace/src/fosa-solution/outils/verifier.sh
if [[ "${1:-}" == --simulation ]]; then
    bash /workspace/src/fosa-solution/simulation/simulation.sh course-sans-carte
fi
' bash "${1:---tests}"
echo "Validation et journaux : $workspace"
