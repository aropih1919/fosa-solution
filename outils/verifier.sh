#!/usr/bin/env bash
# Tests reproductibles dans ROS 2 Jazzy natif ; Docker reste une option isolée.
set -euo pipefail
depot="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --docker ]]; then
    exec bash "$depot/docker/simulation/preparer.sh" "${2:---tests}"
fi
workspace="${FOSA_WS:-$(cd -- "$depot/../.." && pwd)}"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy absent : installer ROS 2 ou utiliser --docker." >&2
    exit 1
fi
if [[ ! -f "$workspace/install/setup.bash" ]]; then
    echo "Workspace non compilé : lancer simulation/simulation.sh preparer." >&2
    exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
set -u
export FOSA_CONFIG_DIR="$depot/caytu_config/config"
unset FOSA_CONFIG_OVERRIDE FOSA_DEPLOYMENT FOSA_NAVIGATION
export PYTHONPATH="$depot/caytu_config:$depot/caytu_course:$depot/caytu_perception:$depot/caytu_nav_solution${PYTHONPATH:+:$PYTHONPATH}"
for script in "$depot/run.sh" "$depot/deployer_robot.sh" "$depot/simulation/simulation.sh" "$depot/outils/"*.sh; do
    bash -n "$script"
done
mkdir -p "$workspace/resultats"
python3 -m pytest -q -ra -p no:cacheprovider \
    --junitxml="$workspace/resultats/tests.xml" \
    "$depot/caytu_config/test" "$depot/caytu_course/test" \
    "$depot/caytu_perception/test" "$depot/caytu_nav_solution/test" \
    "$depot/outils/test" "$depot/outils/test_mesure_rotation.py"
