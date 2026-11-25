# CLAUDE.md

Shared project context for everyone (and every Claude session) working in this repo.

## Purpose
Autoware-based **minimal risk maneuver (MRM) stack** for our Hyundai Ioniq 5 demo car. It runs in hot standby next to a different nominal AD system. On a trigger, the trajectory tracking controller on the dSPACE MicroAutoBox III (MABX) switches from the nominal trajectory to the MRM trajectory. Phase 1: in-lane comfortable stop. Later: pull-over.

We use the full Autoware stack (GNSS localization, perception, planning) and our own vehicle dynamics control. Everything else is built-in Autoware.

## Repo layout
- `/` (this repo): Docker (`docker/`), `repositories/*.repos`, `config/`, `docs/`, this file.
- `src/` holds the cloned Autoware repos. They are upstream code, not ours.
- `src/custom/autoware_custom`: **our MRM code** (separate git repo, schtbs/autoware_custom).
- `src/launcher/autoware_launch`: our fork with the Ioniq 5 vehicle/sensor packages and the MABX bridge (separate git repo, schtbs/autoware_launch).

## Don't search `src/` broadly
`src/` contains the whole Autoware source. Don't grep or glob across it. Look at the specific upstream package you need (e.g. `autoware_mission_planner`, `autoware_behavior_path_planner`, `autoware_velocity_smoother`, `autoware_trajectory_follower_node`) or use targeted reads.

## Docker
Build (from the repo root):
```bash
docker build -t autoware-custom:latest -f docker/custom.Dockerfile .
```
Run:
```bash
docker run --rm -it --net host --privileged --gpus all \
  -e DISPLAY=$DISPLAY -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all \
  -e HOST_UID=$(id -u) -e HOST_GID=$(id -g) -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/work/autoware_data:/home/aw/autoware_data \
  -v $HOME/work/autoware:/home/aw/autoware \
  -v $HOME/work/autoware/config/cyclonedds.xml:/home/aw/autoware/config/cyclonedds.xml \
  -w /home/aw/autoware --runtime=nvidia autoware-custom:latest bash
```
Inside the container:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/home/aw/autoware_data/maps/sample-map-planning \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

## More documentation
- [docs/git-workflow.md](docs/git-workflow.md): branches, PRs, which repo gets which change.
- [docs/build.md](docs/build.md): building and testing.
- [docs/interfaces.md](docs/interfaces.md): topics, trigger, MABX bridge, sim vs car.
