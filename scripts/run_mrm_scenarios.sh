#!/bin/bash
# Runs MRM simulation scenarios, each in a fresh headless container (no GPU, no RViz).
# usage: scripts/run_mrm_scenarios.sh [scenario ...]     default: all scenarios of the file
# env:   SCENARIO_FILE (default: sample map scenarios), AUTOWARE_DATA (default ~/work/autoware_data),
#        IMAGE (default autoware-custom:latest)
# Results: build/mrm_results/<scenario>.txt (KPI table) and .json (raw data)
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
IMAGE=${IMAGE:-autoware-custom:latest}
DATA=${AUTOWARE_DATA:-$HOME/work/autoware_data}
FILE=${SCENARIO_FILE:-/home/aw/autoware/src/custom/autoware_custom/mrm_sim_tools/config/scenarios/sample_map.yaml}
OUT=/home/aw/autoware/build/mrm_results
SRC="source /opt/ros/jazzy/setup.bash; source /home/aw/autoware/install/setup.bash 2>/dev/null"

in_container() {
  docker run --rm --net host --privileged -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
    -v "$DATA":/home/aw/autoware_data -v "$ROOT":/home/aw/autoware -w /home/aw/autoware \
    "$IMAGE" bash -c "$SRC; $1"
}

echo "building MRM packages ..."
in_container "colcon build --symlink-install --packages-select mrm_route_generator mrm_stop_planner mrm_sim_tools mrm_launch > build/mrm_build.log 2>&1; source install/setup.bash 2>/dev/null; true" || exit 1

if [ $# -gt 0 ]; then NAMES="$*"; else NAMES=$(in_container "source install/setup.bash 2>/dev/null; ros2 run mrm_sim_tools run_scenario --file $FILE --list" | grep -v '^\['); fi
rm -rf "$ROOT/build/mrm_results"
for n in $NAMES; do
  echo "--- $n"
  in_container "source install/setup.bash 2>/dev/null; ros2 run mrm_sim_tools run_scenario --file $FILE --name $n --out $OUT" | grep -E "^(===|  (PASS|FAIL|info)|RESULT|trigger)" 
done
echo; echo "reports: $ROOT/build/mrm_results/*.txt"
