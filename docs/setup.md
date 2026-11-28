# Setting up a new PC

How to set up the MRM Autoware workspace on a new machine, build it, and start the stacks in simulation and on the car.
Unless a step says otherwise, **all commands run from the root of this repo** (the `autoware` meta repo).

## Folder layout

```
<parent>/
  autoware/          this repo (meta repo), src/ holds the cloned Autoware repos
  autoware_data/     ML models, demo data, maps (outside git)
```

Examples: `~/work/` on the desktop, `/mnt/data/autoware/` on the toaster. The Docker run command finds `autoware_data` next to the repo (`$(dirname "$PWD")/autoware_data`). To use another folder, set `AUTOWARE_DATA=/some/path`.

## 1. Prerequisites (host)

- Ubuntu 24.04 with an NVIDIA GPU and driver
- Docker with the NVIDIA Container Toolkit (`docker run --gpus all ...` must work)
- `git`, `vcstool` (`vcs`), `ansible`, and the Hugging Face CLI (`hf`, from `pip install -U "huggingface_hub[cli]"`)
- SSH access to GitHub for `schtbs/*`. `autoware_custom` is private.

## 2. Clone

Run once in `<parent>` (the only step that does not run in the repo root):
```bash
mkdir -p autoware_data && git clone https://github.com/schtbs/autoware.git && cd autoware
```
From here on, everything runs in the repo root:
```bash
mkdir -p src && vcs import src < repositories/autoware.repos
```
```bash
vcs import src < repositories/custom.repos
```

## 3. ML models and demo data

Install the repo's Ansible collection once:
```bash
ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml
```
```bash
mkdir -p "$(dirname "$PWD")/autoware_data/ml_models"
```
```bash
ansible-playbook autoware.dev_env.install_dev_env --tags artifacts \
  -e "data_dir=$(dirname "$PWD")/autoware_data/ml_models" --ask-become-pass
```
```bash
ansible-playbook autoware.dev_env.install_dev_env --tags demo_artifacts \
  -e "demo_artifacts__autoware_data_dir=$(dirname "$PWD")/autoware_data" --ask-become-pass
```
The models must match the revisions in `ansible/roles/artifacts/tasks/main.yaml` of this repo. If the launch fails with "No such file ... lidar_centerpoint/base/ml_package.param.yaml", an old `lidar_centerpoint` is in place:
```bash
hf download AutowareFoundation/lidar_centerpoint --revision v4.1 --local-dir "$(dirname "$PWD")/autoware_data/ml_models/lidar_centerpoint"
```

## 4. Map

The KIT campus map (`campus_ost`) is not in git. Copy the folder with `lanelet2_map.osm`, `map_projector_info.yaml` and `pointcloud_map.pcd` to `../autoware_data/maps/autoware_maps/campus_ost`, e.g. from the toaster:
```bash
mkdir -p "$(dirname "$PWD")/autoware_data/maps/autoware_maps" && scp -r <user>@<toaster>:/mnt/data/autoware/autoware_data/maps/autoware_maps/campus_ost "$(dirname "$PWD")/autoware_data/maps/autoware_maps/"
```
The demo step above puts the sample maps in `../autoware_data/maps/` (`sample-map-planning`, `sample-map-rosbag`).

## 5. Build the Docker image

```bash
docker build -f docker/custom.Dockerfile -t autoware-custom:latest .
```
Rebuild the image when the Dockerfile or the dependencies of `src/` change. Code changes only need `colcon build` (step 7).

## 6. Start the container

The container runs in the background with a fixed name. You work in it through a tmux session, so you never need `docker exec ... bash` per terminal, and nothing stops when the SSH connection drops. The ROS domain is part of the container command: pick the block for where you are. Run it in the repo root on the host.

**Car (toaster), MRM or standard stack**: domain 101 (the nominal system and the sensors are in 0, `domain_bridge` copies the sensors over):
```bash
docker run -d --rm --name aw-car --net host --privileged --gpus all --runtime=nvidia \
  -e ROS_DOMAIN_ID=101 -e DISPLAY=${DISPLAY:-:0} -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all \
  -e HOST_UID=$(id -u) -e HOST_GID=$(id -g) -e QT_X11_NO_MITSHM=1 -e ROS_LOG_DIR=/home/aw/autoware/log/ros \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${AUTOWARE_DATA:-$(dirname "$PWD")/autoware_data}":/home/aw/autoware_data \
  -v "$PWD":/home/aw/autoware -w /home/aw/autoware autoware-custom:latest sleep infinity
```
**Simulation** (any PC, also the toaster): domain 98, so it never meets the nominal system (0) or the car's stack (101):
```bash
docker run -d --rm --name aw-sim --net host --privileged --gpus all --runtime=nvidia \
  -e ROS_DOMAIN_ID=98 -e DISPLAY=${DISPLAY:-:0} -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all \
  -e HOST_UID=$(id -u) -e HOST_GID=$(id -g) -e QT_X11_NO_MITSHM=1 -e ROS_LOG_DIR=/home/aw/autoware/log/ros \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${AUTOWARE_DATA:-$(dirname "$PWD")/autoware_data}":/home/aw/autoware_data \
  -v "$PWD":/home/aw/autoware -w /home/aw/autoware autoware-custom:latest sleep infinity
```

**Open a terminal in the container** (the same command reattaches after an SSH drop or from a second terminal; replace `aw-car` with `aw-sim` for the simulation):
```bash
docker exec -it -u aw aw-car tmux new -A -s aw
```
In tmux: `Ctrl-b c` new window, `Ctrl-b 1`..`9` or a mouse click switches, `Ctrl-b "` / `Ctrl-b %` splits, `Ctrl-b d` detaches (everything keeps running). The status bar shows the `ROS_DOMAIN_ID`.

**Stop** the container (all stacks in it stop):
```bash
docker stop aw-car
```
Stop every Autoware container of ours (named `aw-*`, or started from `autoware-custom` without a name):
```bash
{ docker ps -q --filter name=^aw-; docker ps -q --filter ancestor=autoware-custom:latest; } | sort -u | xargs -r docker stop
```

Notes:
- `ROS_LOG_DIR` keeps the ROS logs in `log/ros/` of the repo (git-ignored), so they survive the container (`--rm`). Keep them after a car test.
- `DISPLAY` falls back to `:0`, so over SSH RViz opens on the machine's own screen. RViz starts in full screen (`F11` leaves it; `rviz_fullscreen:=false` starts it windowed).
- Other uses of tmux: several people see the same session at once (laptop over SSH and the car's screen), long jobs (`colcon build`, scenario runs) keep running after you log out, the scroll-back keeps the whole launch log (mouse wheel), and `Ctrl-b :setw synchronize-panes` types into all panes at once.

## 7. Build the workspace (in the container)

First build (whole workspace, takes a long time):
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
After that, only rebuild our packages:
```bash
colcon build --symlink-install --packages-select mrm_route_generator mrm_stop_planner mrm_sim_tools mrm_trigger_udp mrm_launch autoware_launch ioniq5_vehicle_launch
```
New shells (and new tmux windows) source ROS and `install/` by themselves. In the shell where you just built, once:
```bash
source install/setup.bash
```
More on builds and tests: [build.md](build.md).

## 8. Start Autoware

All entry points are in the fork's `autoware_launch/launch/`:

| | Simulation | Car |
|---|---|---|
| Standard Autoware stack | `planning_simulator.launch.xml` | `autoware.launch.xml` |
| MRM stack (hot standby) | `mrm_planning_simulator.launch.xml` | `mrm.launch.xml` |

Each command below is complete: copy it into a tmux window of the container. **Or** start a whole tmux layout with catmux (windows: `stack`, `trigger` with the trigger command ready (press Up, Enter), `status` (`/mrm/status`), `bag` with the rosbag command ready, `shell`). The session files are in `config/catmux/`.

### In simulation (container `aw-sim`)

MRM stack: a fake nominal system drives the car along `path_file`, Autoware and the MRM run in standby. Set the initial pose in RViz and switch to autonomous mode (the route is generated automatically), then trigger (section 9).

Without the MicroAutoBox:
```bash
ros2 launch autoware_launch mrm_planning_simulator.launch.xml map_path:=/home/aw/autoware_data/maps/autoware_maps/campus_ost vehicle_model:=ioniq5_vehicle sensor_model:=ioniq5_sensor_kit path_file:=$(ros2 pkg prefix mrm_sim_tools)/share/mrm_sim_tools/config/paths/campus_ost_lanelet_1903750.yaml
```
With the MicroAutoBox (bench test: trajectory 44444, status 44445 and trigger 44455 go to 10.0.10.20 exactly as on the car; **only with the car parked and the MABX not engaged**, the trajectory follows the simulated car):
```bash
ros2 launch autoware_launch mrm_planning_simulator.launch.xml map_path:=/home/aw/autoware_data/maps/autoware_maps/campus_ost vehicle_model:=ioniq5_vehicle sensor_model:=ioniq5_sensor_kit path_file:=$(ros2 pkg prefix mrm_sim_tools)/share/mrm_sim_tools/config/paths/campus_ost_lanelet_1903750.yaml autobox_ip:=10.0.10.20
```
The same as a catmux layout, without / with the MicroAutoBox:
```bash
catmux_create_session -L default config/catmux/sim_mrm.yaml
```
```bash
catmux_create_session -L default config/catmux/sim_mrm.yaml --overwrite mabx=true
```

Standard stack: set the initial pose and the goal in RViz, then engage:
```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/home/aw/autoware_data/maps/autoware_maps/campus_ost vehicle_model:=ioniq5_vehicle sensor_model:=ioniq5_sensor_kit vehicle_id:=ioniq5
```
```bash
catmux_create_session -L default config/catmux/sim_standard.yaml
```

Headless scenarios with KPIs (on the host, in the repo root, one fresh container per scenario):
```bash
scripts/run_mrm_scenarios.sh
```

### On the car (container `aw-car`)

1. Make sure no other stack sends to the MicroAutoBox: `docker ps` must not show a container of the old stack (the old Autoware or DDS router from `/home/inkmi5/`; stop it by its name). Keep `ouster_lidar_node` running, it publishes `/ouster/points` in domain 0.
2. Start the container `aw-car` (section 6, domain 101). The vehicle interface starts `domain_bridge`, which copies the sensor topics from domain 0 into 101.
3. Start **one** of the stacks.

MRM stack (hot standby, sends `/mrm/trajectory` and the MRM trigger):
```bash
ros2 launch autoware_launch mrm.launch.xml map_path:=/home/aw/autoware_data/maps/autoware_maps/campus_ost vehicle_model:=ioniq5_vehicle sensor_model:=ioniq5_sensor_kit vehicle_id:=ioniq5 pose_source:=anavs twist_source:=anavs lateral_controller_mode:=pure_pursuit pose_initializer_stop_check:=false
```
```bash
catmux_create_session -L default config/catmux/car_mrm.yaml
```
Standard stack (drives the car, sends `/planning/trajectory`):
```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=/home/aw/autoware_data/maps/autoware_maps/campus_ost vehicle_model:=ioniq5_vehicle sensor_model:=ioniq5_sensor_kit vehicle_id:=ioniq5 pose_source:=anavs twist_source:=anavs lateral_controller_mode:=pure_pursuit pose_initializer_stop_check:=false
```
```bash
catmux_create_session -L default config/catmux/car_standard.yaml
```

**Dry run** (nothing reaches the MicroAutoBox, all UDP goes to localhost):
```bash
ros2 launch autoware_launch mrm.launch.xml map_path:=/home/aw/autoware_data/maps/autoware_maps/campus_ost vehicle_model:=ioniq5_vehicle sensor_model:=ioniq5_sensor_kit vehicle_id:=ioniq5 pose_source:=anavs twist_source:=anavs lateral_controller_mode:=pure_pursuit pose_initializer_stop_check:=false autobox_ip:=127.0.0.1
```
```bash
catmux_create_session -L default config/catmux/car_mrm.yaml --overwrite autobox_ip=127.0.0.1
```
To watch the packets on the host:
```bash
sudo tcpdump -i lo -n udp port 44444 or udp port 44445 or udp port 44455
```

`mrm.launch.xml` refuses to start in domain 0. UDP to the MicroAutoBox (10.0.10.20), details in [interfaces.md](interfaces.md):

| Port | What | Stack |
|---|---|---|
| 44444 | trajectory (31 points: 6 behind the car, the car, 24 ahead, 0.25-1 m apart by curvature) | both |
| 44445 | trajectory status (up to date / valid) | both |
| 44455 | MRM trigger (50 Hz, latched) | MRM |

## 9. Trigger the MRM

In the MRM stacks (simulation and car), either:
- press **"Set Emergency"** in RViz's Autoware state panel. The MRM launch files point it to the MRM trigger, so it does not trigger Autoware's own emergency stop. The button turns red ("Clear Emergency"). Clearing is refused, because the MRM is latched until the stack restarts.
- or in a tmux window of the same container (catmux: window `trigger`, press Up, Enter):
  ```bash
  ros2 topic pub --times 3 /mrm/trigger std_msgs/msg/Bool "{data: true}"
  ```

On the car, `mrm_trigger_udp_sender` then sends `trigger = 1` to the MicroAutoBox (10.0.10.20:44455), which switches to the MRM trajectory. To start over, restart `mrm.launch.xml` (`Ctrl-C` in the `stack` window, Up, Enter).

In RViz the MRM trajectory (sent to the MABX) is the **orange** line with orange speed bars; Autoware's hot-standby plan is the wide **translucent cyan** band.
