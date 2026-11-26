# MRM stack on the Ioniq 5

State (2026-11-26): **the standard Autoware stack (`autoware.launch.xml`, domain 101) drove the Ioniq 5 closed loop through the MicroAutoBox, with everything running on the car.** The MRM stack (`mrm.launch.xml`) was started on the toaster with the real sensors (dry run, car parked, see "ROS network") but has not been driven yet: no drive with `/mrm/trajectory` to the MicroAutoBox and no real trigger. The MRM behavior was checked in the planning simulator with the Ioniq 5 vehicle model on the campus map, or in isolation.

## What runs where

```
nominal AD system ──────────────────────────────────────────► MicroAutoBox ──► vehicle
                                                                  ▲   │
   MRM PC (own ROS_DOMAIN_ID)                                     │   │ trigger (UDP)
   Autoware full stack ─ /planning/trajectory ─► mrm_stop_planner │   │
        (sensing, localization, perception,          │            │   │
         planning with avoidance)                    ▼            │   ▼
   mrm_route_generator                       /mrm/trajectory ─► bridge ─UDP─┘   mrm_trigger_udp
```

- The MRM Autoware runs the full stack as on the car today. Its own control and vehicle interface stay as they are. Only the trajectory sent to the MicroAutoBox changes.
- `mrm_stop_planner` turns Autoware's plan into the stop trajectory (starts at the rear axle, stop profile, avoidance by Autoware).
- `vehicle_status_publisher.py` reports AUTONOMOUS and DRIVE permanently, so Autoware plans as if engaged while the nominal system drives.

## Start

Stop the old stack first. It runs on domain 0 and its `autobox_udp_sender` also talks to the MicroAutoBox: `docker stop angry_kowalevski practical_hertz` (old Autoware in `/home/inkmi5/autoware`, old DDS router in `/home/inkmi5/oussama_ws`). Keep `ouster_lidar_node` running, it publishes `/ouster/points` in domain 0.

All car and simulation entry points are in the fork's `autoware_launch/launch/`:

| | simulation | real vehicle |
|---|---|---|
| standard AD stack | `planning_simulator.launch.xml` | `autoware.launch.xml` |
| MRM stack | `mrm_planning_simulator.launch.xml` | `mrm.launch.xml` |

The MRM launch files include `mrm_launch/mrm_core.launch.xml` (route generator + stop planner, the same in simulation and on the car) from autoware_custom.

The car arguments (same for both stacks):
```bash
CAR_ARGS="map_path:=/home/aw/autoware_data/maps/autoware_maps/campus_ost vehicle_model:=ioniq5_vehicle sensor_model:=ioniq5_sensor_kit vehicle_id:=ioniq5 pose_source:=anavs twist_source:=anavs lateral_controller_mode:=pure_pursuit pose_initializer_stop_check:=false"
```
- `pose_source:=anavs twist_source:=anavs`: pose and twist both come from the ANavS (the twist is published on `/fzi/twist_with_covariance_stamped`). `ioniq5_sensor_kit_launch` relays them straight into the EKF, so no Autoware estimator (NDT, gyro_odometer) runs. Any value that is not an Autoware estimator has this effect, the earlier command line called them `AnaVs` / `FZI`.
- `pose_initializer_stop_check:=false`: see "Localization".

From the repo root on the toaster, in a container started with the CLAUDE.md run command plus `-e ROS_DOMAIN_ID=101`:
```bash
ros2 launch autoware_launch autoware.launch.xml $CAR_ARGS     # standard AD stack
ros2 launch autoware_launch mrm.launch.xml $CAR_ARGS          # MRM stack (next to the nominal system)
```
Both send their trajectory to the MicroAutoBox (`10.0.10.20:44444`), the standard stack `/planning/trajectory`, the MRM stack `/mrm/trajectory`. Add `autobox_ip:=127.0.0.1` for a **dry run** (packets go to localhost). Packets only go out once there is a route and a trajectory.
`mrm.launch.xml` refuses to start when `ROS_DOMAIN_ID` equals `nominal_domain` (default 0).
The vehicle interface (`ioniq5_vehicle_launch/vehicle_interface.launch.xml`) starts the domain bridge for the sensor topics when `ROS_DOMAIN_ID` differs from `sensor_domain` (default 0), see "ROS network".

Differences from the normal car launch:

| | normal | MRM stack |
|---|---|---|
| Trajectory sent to the MicroAutoBox | `/planning/trajectory` | `/mrm/trajectory` (`trajectory_input_topic`, fork branch `feature/bridge-input-topic`) |
| `enable_all_modules_auto_mode` | `false` | `true` (nobody approves avoidance or lane change in standby) |
| Route | set by hand | `mrm_route_generator` along the current lane |

## Interfaces to the MicroAutoBox

| Direction | What | Status |
|---|---|---|
| MRM PC to MABX | UDP packet with the trajectory (`autobox_udp_sender`, `10.0.10.20:44444`) | existing, unchanged |
| MABX to MRM PC | MRM trigger over UDP, port 45000 (`mrm_trigger_udp`) | **packet format is a placeholder** (1 byte, nonzero = trigger), see `mrm_trigger_udp/packet.py` |

Behavior of the trigger: the first `True` is latched by all MRM nodes (route frozen, stop point fixed). A `False` does not undo it, the MRM stack has to be restarted. A lost link never triggers (nothing is published without a packet).

## ROS network

The nominal system is ROS 2 on domain 0 on the same network. Generic Autoware topics (`/planning/*`, `/control/*`, `/localization/*`, ...) must not meet, so the MRM stack runs in **domain 101**.
`ioniq5_vehicle_launch/config/domain_bridge.yaml` (ROS 2 `domain_bridge`, started by `vehicle_interface.launch.xml` for the standard and the MRM stack) copies only the raw sensor inputs from domain 0 into 101, nothing goes back:

| Topic | Used by |
|---|---|
| `/ouster/points` | `ouster_format_converter` (LiDAR) |
| `/sensor/camera_f/undistorted/image`, `.../camera_info` | camera6 relay, YOLOX (the other cameras are not bridged) |
| `/geo` | `gnss_poser` |
| `/hi5/anavs/imu` | IMU relay |
| `/fzi/twist_with_covariance_stamped` (ANavS twist) | twist relays, `twist_to_velocity_report` |
| `/hi5/tf/center_rear_axle_ned` | `tf_to_gnssinsorient_converter` |
| `/steering_angle` | `vehicle_status_publisher` |

QoS follows the publisher in domain 0. `/tf` and `/tf_static` are not bridged, the MRM Autoware publishes its own TF tree. Only UDP crosses to the MicroAutoBox.

Checked on the toaster (2026-11-26, car parked): all bridged inputs arrive in 101 at their domain-0 rates (LiDAR 10 Hz, GNSS/IMU/twist/TF/steering 100 Hz; camera_f had no publisher in domain 0 at that time), localization initializes from GNSS, EKF 50 Hz, perception objects 10 Hz, domain 0 shows none of the MRM stack's topics, domain_bridge uses about 25 % of one core. Planning and `/mrm/trajectory` need the car on a lane with good GNSS (it was in the garage, 12 m off the map's lanelets).

## Localization (as in the earlier car integration)

No NDT (no usable pointcloud map) and no gyro_odometer: `ioniq5_sensor_kit_launch` relays the ANavS pose and the ANavS twist (topic `/fzi/twist_with_covariance_stamped`) straight into the EKF, and the pose initializer starts from GNSS.
The earlier integration switched NDT and the stop check off in `pose_initializer.param.yaml` (`[oussama]` comments, now in `autoware_localization_config`). Upstream's `pose_initializer.launch.xml` now sets `ndt_enabled` and `stop_check_enabled` from launch arguments after loading that file, so those file values no longer apply. The car arguments turn them off instead (`pose_source:=anavs`, and `pose_initializer_stop_check:=false`, an argument added in `component_localization.launch.xml`). With the stop check on, initialization failed: its 1 mm/s threshold is below the ANavS twist at standstill (about 3 mm/s, seen in the garage with poor GNSS).
The team's planning tuning (max_vel 8 m/s, avoidance margins, obstacle stop/slow down/cruise, surround checker) was carried over into `autoware_planning_config` in the merge and is in effect.

## Perception on the car (differences from upstream)

- **Ouster azimuth:** `ouster_format_converter.py` writes the azimuth in the scan direction (clockwise). Upstream's ring outlier filter needs it increasing along a ring and otherwise drops every point (the cloud was empty, no obstacle reactions).
- **Crop box above the car** (`ouster_node_container.py`, `crop_box_filter_above_vehicle`): vehicle length, body width + 0.4 m, z 1.4-2.3 m. Floating points over the hood (sensor rig / reflections) had become a pedestrian on the hood that blocked the start (surround obstacle checker). Arguments: `above_vehicle_crop_box_min_z`, `_max_z`, `_lateral_margin`.
- **Time-series obstacle filter** (`component_perception.launch.xml`, `use_obstacle_segmentation_time_series_filter`, default `true`): the occupancy grid based outlier filter of autoware_launch 0.52.0, which the old stack drove with; upstream dropped it (#1942).
- **`perception_mode`** (default `lidar`): `camera_lidar_fusion` adds ROI cluster fusion with the front camera (YOLOX on camera6). It starts and projects (camera frames `camera_X` in the URDF, ROI offset -0.01 s), but camera classes on the objects are **not verified yet**. The old stack's `camera_lidar_fusion` never got camera classes either (it expected ROIs from cameras 0-5, and the frame `camera_f` was missing), so `lidar` matches its driving behavior.
- Compared with the old stack (`/home/inkmi5/autoware`, autoware_launch 0.52.0): all team parameters (planning, control, localization, system, vehicle) are in effect, including the static/moving thresholds (`is_moving_threshold_velocity` 1.0, `moving_object_speed_threshold` 1.0, `th_moving_speed` 1.0 / `th_moving_time` 2.0). Open: one emergency brake in the test drive for an obstacle taken as a crossing car; cause not recorded.

## Data

The ML models must match the revisions in `ansible/roles/artifacts/tasks/main.yaml` of **this** repo (e.g. `lidar_centerpoint` v4.1 with the `base/`, `tiny/`, ... folders). The deprecated checkout in `/home/inkmi5/autoware` pins `lidar_centerpoint` v3.0 (flat layout, `deploy_metadata.yaml`: `version: v3.0`). The current launch then fails with "No such file ... lidar_centerpoint/base/ml_package.param.yaml". Note that the role downloads to `~/autoware_data` by default, which on the toaster is the old data folder, not `/mnt/data/autoware/autoware_data`. Fix:
```bash
hf download AutowareFoundation/lidar_centerpoint --revision v4.1 --local-dir ../autoware_data/ml_models/lidar_centerpoint
```

## Map

The route generator reads the map directory (`lanelet2_map.osm` + `map_projector_info.yaml`). Supported projectors: `MGRS`, `LocalCartesian`, `TransverseMercator`. Other types fail at start with a clear message. `LocalCartesian` is loaded with lanelet2's `LocalCartesianProjector` (as in Autoware's `map_projection_loader`), a `yaw` entry in `map_origin` is not supported.

## Checked in simulation (Ioniq 5 on the campus map)

See `scripts/run_mrm_scenarios.sh` and `docs/requirements.md`. The `campus_*` scenarios are in `mrm_sim_tools/config/scenarios/campus_ost.yaml`.
`campus_straight` and `campus_curve` (trigger inside the curve after a 90 deg turn and a U-turn) pass all KPIs except the tracking overshoot of the deceleration limits (MRM-006, limits TBD): longest gap between trajectories 0.06 s, trajectory 0.001 to 0.006 m from the ego.
Found and fixed with this map:
- Autoware's mission planner takes the **shortest** route to the goal, not the lanes the route generator chose. The route generator now passes a via point in every lanelet in between. Without it the plan left the ego's path and `/mrm/trajectory` was missing for 50 s.
- In a U-turn the closest plan segment by position can be the opposite leg. The stop planner now projects the ego with its heading (as the Autobox bridge does).
- The fake nominal and the scenario runner had the same problem and now search for the ego near its last position.
- A re-route can be refused while Autoware is in a turn or intersection ("planner is not in lane following"). The route generator retries with backoff, the 300 m route with 120 m minimum remaining leaves room for that.

## Not checked, needs the car

0. ~~That `trajectory_input_topic` reaches the bridge.~~ Checked on the toaster: `ros2 param get /autobox_trajectory_bridge input_topic` = `/mrm/trajectory` (on the fork's `main`).
0. ~~MicroAutoBox link with the standard stack.~~ Driven closed loop on 2026-11-26 with `autoware.launch.xml`.

Next car day, in this order:

1. `mrm.launch.xml` in standby while the nominal drives: `/mrm/trajectory` reaches the bridge and the MicroAutoBox; record a rosbag (`/mrm/trajectory`, `/mrm/status`, `/localization/kinematic_state`, `/perception/object_recognition/objects`). Check `/mrm/status` for `no_objects` (the object check needs the perception output) and for unexpected `object_*` warnings next to parked cars.
2. Latency chain planner to MicroAutoBox (MRM-004: age <= 0.1 s) and the end of the trajectory window at the MABX in the last 3 s of a stop (MRM-007). The simulator only measures up to the ROS topic.
3. Difference between `/localization/kinematic_state` (EKF) and the raw A-ROX pose that the MABX uses (MRM-001, drive check).
4. Rear-axle reference of the pose (MRM-003, lever arm confirmed, drive check open).
5. Real trigger packet and the reaction of the MABX at standstill.
6. Load: Autoware perception plus the Python MRM nodes on the MRM PC at 20 Hz.
7. Built-in Autoware MRM handler: it still runs in the MRM Autoware (comfortable stop is enabled in the fork). It cannot reach the vehicle, because no Autoware command is sent to it. **But its comfortable stop ends up in `/planning/trajectory`, and the MRM stack uses the stop points of that plan**: in the simulator Autoware's planning validator rejected its own lagging hot-standby plan ("deceleration is too high"), the built-in comfortable stop ran for 8.5 s, and a trigger in that time made the MRM brake with 2.5 m/s2 (docs/requirements.md, open question 3). On the car: watch `/api/fail_safe/mrm_state` and `planning_validator` messages in standby.
8. Map: lanes of the real route (width, stop lines, traffic lights in `campus_ost`). The sample-map virtual traffic light stopped Autoware's plan in the simulator, such elements on the real map need a look.
