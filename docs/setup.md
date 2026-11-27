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

```bash
docker run --rm -it --net host --privileged --gpus all \
  -e DISPLAY=${DISPLAY:-:0} -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all \
  -e HOST_UID=$(id -u) -e HOST_GID=$(id -g) -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${AUTOWARE_DATA:-$(dirname "$PWD")/autoware_data}":/home/aw/autoware_data \
  -v "$PWD":/home/aw/autoware \
  -w /home/aw/autoware --runtime=nvidia autoware-custom:latest bash
```
- On the car (toaster), add `-e ROS_DOMAIN_ID=101` (see "On the car"). Without it the container uses domain 0, which belongs to the nominal AD system and the sensors.
- `DISPLAY` falls back to `:0`, so over SSH RViz opens on the machine's own screen.
- A second shell in the same container: `docker exec -it <container> bash` (container name from `docker ps`).

## 7. Build the workspace (in the container)

First build (whole workspace, takes a long time):
```bash
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
After that, only rebuild our packages:
```bash
colcon build --symlink-install --packages-select mrm_route_generator mrm_stop_planner mrm_sim_tools mrm_trigger_udp mrm_launch autoware_launch ioniq5_vehicle_launch
```
Then, in every shell:
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```
More on builds and tests: [build.md](build.md).

## 8. Start Autoware

All entry points are in the fork's `autoware_launch/launch/`:

| | Simulation | Car |
|---|---|---|
| Standard Autoware stack | `planning_simulator.launch.xml` | `autoware.launch.xml` |
| MRM stack (hot standby) | `mrm_planning_simulator.launch.xml` | `mrm.launch.xml` |

### In simulation (any PC)

Standard stack, Ioniq 5 on the campus map. Set the initial pose and the goal in RViz, then engage:
```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/home/aw/autoware_data/maps/autoware_maps/campus_ost vehicle_model:=ioniq5_vehicle sensor_model:=ioniq5_sensor_kit vehicle_id:=ioniq5
```
MRM stack: a fake nominal system drives the car along `path_file`, and Autoware plus the MRM run in standby. Set the initial pose in RViz and switch to autonomous mode (the route is generated automatically), then trigger the MRM (see "Trigger the MRM"):
```bash
ros2 launch autoware_launch mrm_planning_simulator.launch.xml map_path:=/home/aw/autoware_data/maps/autoware_maps/campus_ost vehicle_model:=ioniq5_vehicle sensor_model:=ioniq5_sensor_kit path_file:=$(ros2 pkg prefix mrm_sim_tools)/share/mrm_sim_tools/config/paths/campus_ost_lanelet_1903750.yaml
```
Headless scenarios with KPIs (on the host, in the repo root, one fresh container per scenario):
```bash
scripts/run_mrm_scenarios.sh
```

### On the car (toaster)

1. Stop the old stack. It runs in domain 0 and also sends to the MicroAutoBox. Keep `ouster_lidar_node` running.
   ```bash
   docker stop angry_kowalevski practical_hertz
   ```
2. Start the container with `-e ROS_DOMAIN_ID=101` (step 6). The vehicle interface starts `domain_bridge`, which copies the sensor topics from domain 0 into 101.
3. In the container, set the car arguments (the same for both stacks):
   ```bash
   CAR_ARGS="map_path:=/home/aw/autoware_data/maps/autoware_maps/campus_ost vehicle_model:=ioniq5_vehicle sensor_model:=ioniq5_sensor_kit vehicle_id:=ioniq5 pose_source:=anavs twist_source:=anavs lateral_controller_mode:=pure_pursuit pose_initializer_stop_check:=false"
   ```
4. Start **one** of the stacks:
   ```bash
   ros2 launch autoware_launch autoware.launch.xml $CAR_ARGS
   ```
   ```bash
   ros2 launch autoware_launch mrm.launch.xml $CAR_ARGS
   ```

The standard stack sends `/planning/trajectory` to the MicroAutoBox. The MRM stack sends `/mrm/trajectory` and the MRM trigger. `mrm.launch.xml` refuses to start in domain 0.

**Dry run** (nothing reaches the MicroAutoBox, all UDP goes to localhost): append `autobox_ip:=127.0.0.1`. To watch the packets on the host:
```bash
sudo tcpdump -i lo -n udp port 44444 or udp port 44445 or udp port 44455
```

UDP to the MicroAutoBox (10.0.10.20), details in [interfaces.md](interfaces.md):

| Port | What | Stack |
|---|---|---|
| 44444 | trajectory (31 points) | both |
| 44445 | trajectory status (up to date / valid) | both |
| 44455 | MRM trigger (50 Hz, latched) | MRM |

## 9. Trigger the MRM

In the MRM stacks (simulation and car), either:
- press **"Set Emergency"** in RViz's Autoware state panel. The MRM launch files point it to the MRM trigger, so it does not trigger Autoware's own emergency stop. The button turns red ("Clear Emergency"). Clearing is refused, because the MRM is latched until the stack restarts.
- or publish the trigger from a shell in the same container (on the car this is domain 101, already set in the container):
  ```bash
  ros2 topic pub --times 3 /mrm/trigger std_msgs/msg/Bool "{data: true}"
  ```

On the car, `mrm_trigger_udp_sender` then sends `trigger = 1` to the MicroAutoBox (10.0.10.20:44455), which switches to the MRM trajectory. To start over, restart `mrm.launch.xml`.
