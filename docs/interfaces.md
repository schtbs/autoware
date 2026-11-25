# Interfaces

## Architecture: one MRM core, two platform adapters
The MRM core is identical in simulation and on the car. Only the adapters differ.

```
map, GNSS localization, perception, /mrm/trigger
   -> mrm_route_generator -> Autoware planning (built-in) -> /planning/trajectory
   -> mrm_stop_planner -> /mrm/trajectory (+ /mrm/status)

SIM adapter:  fake_nominal -> sim_trajectory_mux (switches on trigger) -> Autoware control -> simulator
CAR adapter:  autobox_trajectory_bridge (input /mrm/trajectory) -> autobox_udp_sender -> MABX
```

## Core interfaces
| Topic | Type | Direction |
|---|---|---|
| `/localization/kinematic_state` | `nav_msgs/Odometry` | in (GNSS-based ego pose and speed) |
| `/planning/trajectory` | `autoware_planning_msgs/Trajectory` | in (Autoware planning output) |
| `/mrm/trigger` | `std_msgs/Bool` (to be confirmed) | in (sim: script/manual, car: from the MABX) |
| `/mrm/trajectory` | `autoware_planning_msgs/Trajectory` | out |
| `/mrm/status` | tbd (standby / active / stopped) | out, diagnostics only |

The core must not contain sim-only or car-only code. All new topics use the `/mrm/` prefix.

## Car side (existing, in `src/launcher/autoware_launch/vehicle/ioniq5_vehicle_launch`)
- `autobox_trajectory_bridge.py`: Autoware trajectory -> resampled and smoothed `ControllerTrajectory` on `/HI5/autoware_controller_trajectory`. For the MRM its `input_topic` is `/mrm/trajectory` (default today: `/planning/trajectory`).
- `autobox_udp_sender.py`: 31-point UDP packet to the MABX (10.0.10.20:44444).
- `vehicle_status_publisher.py`: reports AUTONOMOUS/DRIVE permanently, so Autoware plans as if engaged while the MABX drives.
- Localization is GNSS-based (`sensor_kit/ioniq5_sensor_kit_launch`). We assume it is always available.
- The built-in Autoware MRM handler must not interfere with the MRM trajectory (see `config/system/mrm_handler`).

## Network isolation on the car
The nominal AD system is ROS 2 on the same network. Run the MRM Autoware in its own `ROS_DOMAIN_ID` / CycloneDDS config (`config/cyclonedds.xml`) so that generic Autoware topics (`/planning/*`, `/control/*`, ...) don't collide. Only the trigger and the bridge cross over.

## Route generator (`mrm_route_generator`)
- The nominal goal is unknown, so the route follows the current lane (straight successors preferred) for `route_length_m` (300 m default) and ends `goal_back_m` before the end of the last lanelet.
- The map is read from `map_path` (lanelet2 file + `map_projector_info.yaml`). Only the `MGRS` projector is supported so far, other types raise an error.
- First route: `/api/routing/set_route_points`. Later changes: `/api/routing/change_route_points` (Autoware answers "already set" to `set_route_points`). Re-routing while autonomous works in the planning simulator. If a change is refused, it falls back to `clear_route` + `set_route_points`. Autoware refuses `clear_route` while the route is in use, so this fallback has not worked so far in simulation.
- A new route is requested when there is none, the route state is not `SET`, the ego left the route, or less than `min_remaining_m` is left. If the map ends (the goal would not change), the old route stays.
- After the trigger (`/mrm/trigger`) the route is frozen.
- Several parallel lanelets often contain the ego (forks, adjacent lanes). The ego lanelet is chosen by containment, distance and heading; a lanelet on the current route wins among equally good candidates.

## Stop planner (`mrm_stop_planner`)
- Input: `/planning/trajectory` (Autoware's plan), `/localization/kinematic_state`, `/mrm/trigger`. Output: `/mrm/trajectory`, `/mrm/status` (`diagnostic_msgs/DiagnosticStatus`: states `standby`, `active`, `stopped`, `invalid`).
- Runs at 20 Hz on the latest ego pose, so the first trajectory point is the ego rear axle in every cycle (MRM-001), also in standby.
- Geometry: Autoware's planned path (lane following, avoidance, lane change) with a cubic lateral offset from the ego pose back onto that path (MRM-002), sized for `blend_lat_acc_mps2` (0.5). At 8 m/s, 0.8 m offset plus 4 deg heading error need about 49 m, which is longer than the stop distance, so the car may stop before it is back on Autoware's path.
- Speed: only the jerk-limited stop profile (`decel_mps2` 1.0, `jerk_mps3` 1.0). **Autoware's planned speeds are not used**: in hot standby Autoware does not control the car, so its speed at the ego lags or leads the real speed (it replans only if the deviation is more than 5.5 m/s). A first version that used them as a limit produced a deceleration burst and speed bumps in the first meters.
- Autoware's **stop points** are used (a plan point <= `plan_stop_speed_mps` that is at least `plan_stop_min_dist_m` ahead, e.g. an obstacle). If the plan stops before the comfortable stop, the planner stops there with up to `plan_max_decel_mps2` (2.5) and `plan_max_jerk_mps3` (1.5), Autoware's absolute limits. With 1.0 m/s3 a stop from 8 m/s needs 22.7 m even at 2.5 m/s2, with 1.5 m/s3 19.5 m. If even that is not enough, the status says so and the maximum is used.
- The stop point itself is always a trajectory sample, and speeds never rise along the trajectory.
- On the trigger the stop profile and the stop point are latched, so the stop point does not creep while braking. A later obstacle may only move the stop point closer.
- Standby with an invalid input (no ego state, no plan, plan older than `max_input_age_s`, ego more than `max_offset_m` off the path): nothing is published (MRM-004). After the trigger the last plan is used.
- Scenario results (simulator, sample vehicle, steady 8 m/s): planned peak deceleration 1.005 m/s2, measured 1.13 m/s2 and jerk 1.1 m/s3. If the trigger comes while the nominal still accelerates (+1 m/s2), the measured peak is 1.6 m/s2 and 3.7 m/s3: the controller sees a step from +1 to -1 m/s2 at the switch. Starting the profile at the current acceleration instead of 0 would soften that (open decision).
