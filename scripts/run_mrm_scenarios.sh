#!/bin/bash
# Runs MRM simulation scenarios, each in a fresh headless container (no GPU, no RViz).
# usage: scripts/run_mrm_scenarios.sh [--gui] [scenario ...]     default: all scenarios of the file
# --gui: open RViz (Autoware plan + /mrm/trajectory in magenta) to watch the scenarios one after the other;
#        needs a display (DISPLAY) and the nvidia runtime. Each scenario starts 25 s after RViz, the
#        simulator stays up for 20 s at the end. Stop with: docker stop mrm-gui
# env:   SCENARIO_FILE (default: sample map scenarios), AUTOWARE_DATA (default ~/work/autoware_data),
#        IMAGE (default autoware-custom:latest)
# Results: build/mrm_results/<scenario>.txt (KPI table) and .json (raw data)
set -u
GUI=0
if [ "${1:-}" = "--gui" ]; then GUI=1; shift; fi
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

in_container_gui() {   # same display / GPU flags as the normal Autoware container
  docker run --rm --name mrm-gui --net host --privileged --gpus all --runtime=nvidia \
    -e DISPLAY="$DISPLAY" -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all -e QT_X11_NO_MITSHM=1 \
    -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$DATA":/home/aw/autoware_data -v "$ROOT":/home/aw/autoware -w /home/aw/autoware \
    "$IMAGE" bash -c "$SRC; $1"
}

echo "building MRM packages ..."
in_container "colcon build --symlink-install --packages-select mrm_route_generator mrm_stop_planner mrm_sim_tools mrm_launch > build/mrm_build.log 2>&1; source install/setup.bash 2>/dev/null; true" || exit 1

if [ $# -gt 0 ]; then NAMES="$*"; else NAMES=$(in_container "source install/setup.bash 2>/dev/null; ros2 run mrm_sim_tools run_scenario --file $FILE --list" | grep -v '^\['); fi
mkdir -p "$ROOT/build/mrm_results"
for n in $NAMES; do
  echo "--- $n"
  if [ "$GUI" = 1 ]; then
    in_container_gui "source install/setup.bash 2>/dev/null; ros2 run mrm_sim_tools run_scenario --file $FILE --name $n --out $OUT --gui" | grep -E "^(===|  (PASS|FAIL|info)|RESULT|trigger|scenario finished)"
    continue
  fi
  rm -f "$ROOT/build/mrm_results/$n".*
  in_container "source install/setup.bash 2>/dev/null; ros2 run mrm_sim_tools run_scenario --file $FILE --name $n --out $OUT" | grep -E "^(===|  (PASS|FAIL|info)|RESULT|trigger)" 
done
echo; echo "reports: $ROOT/build/mrm_results/*.txt"
