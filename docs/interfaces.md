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
| `/perception/object_recognition/objects` | `autoware_perception_msgs/PredictedObjects` | in (static objects for the footprint check, MRM-008) |
| `/mrm/status` | `diagnostic_msgs/DiagnosticStatus` (see below) | out, diagnostics only |

The core must not contain sim-only or car-only code. All new topics use the `/mrm/` prefix.

## Car side (existing, in `src/launcher/autoware_launch/vehicle/ioniq5_vehicle_launch`)
- `autobox_trajectory_bridge.py`: Autoware trajectory -> resampled and smoothed `ControllerTrajectory` on `/HI5/autoware_controller_trajectory`. For the MRM its `input_topic` is `/mrm/trajectory` (default today: `/planning/trajectory`).
- `autobox_udp_sender.py`: 31-point UDP packet to the MABX (10.0.10.20:44444).
- `vehicle_status_publisher.py`: reports AUTONOMOUS/DRIVE permanently, so Autoware plans as if engaged while the MABX drives.
- Localization is GNSS-based (`sensor_kit/ioniq5_sensor_kit_launch`). We assume it is always available.
- The built-in Autoware MRM must not interfere with the MRM trajectory: in the MRM stack its comfortable stop does not limit the planned velocity (`autoware_mrm_comfortable_stop_to_planning:=false`, set by `mrm.launch.xml` / `mrm_planning_simulator.launch.xml`).

## Network isolation on the car
The nominal AD system is ROS 2 on the same network. Run the MRM Autoware in its own `ROS_DOMAIN_ID` / CycloneDDS config (`config/cyclonedds.xml`) so that generic Autoware topics (`/planning/*`, `/control/*`, ...) don't collide. Only the trigger and the bridge cross over.

## Route generator (`mrm_route_generator`)
- The nominal goal is unknown, so the route follows the current lane (straight successors preferred) for `route_length_m` (300 m default) and ends `goal_back_m` before the end of the last lanelet.
- The map is read from `map_path` (lanelet2 file + `map_projector_info.yaml`). Projectors: `MGRS`, `LocalCartesian`, `TransverseMercator` (see `docs/car-integration.md`).
- First route: `/api/routing/set_route_points`. Later changes: `/api/routing/change_route_points` (Autoware answers "already set" to `set_route_points`). Re-routing while autonomous works in the planning simulator.
- A refused change (e.g. "planner is not in lane following" in a turn) is retried with backoff (`retry_interval_s` 3 s, doubling, at most `retry_max_interval_s` 6 s); the current route stays in use meanwhile. The route is **never cleared**: Autoware refuses `clear_route` while the car moves in autonomous mode under Autoware control (`autoware_default_adapi`, `routing.cpp`), which is always the case in hot standby (`vehicle_status_publisher` reports AUTONOMOUS), and a cleared route would stop `/mrm/trajectory` (MRM-004). The earlier `clear_route` fallback never worked and was removed.
- A new route is requested when there is none, the route state is not `SET`, the ego left the route, or less than `min_remaining_m` is left. If the map ends (the goal would not change), the old route stays.
- After the trigger (`/mrm/trigger`) the route is frozen.
- Several parallel lanelets often contain the ego (forks, adjacent lanes). The ego lanelet is chosen by containment, distance and heading; a lanelet on the current route wins among equally good candidates.

## Stop planner (`mrm_stop_planner`)
- Input: `/planning/trajectory` (Autoware's plan), `/localization/kinematic_state`, `/perception/object_recognition/objects`, `/mrm/trigger`. Output: `/mrm/trajectory`, `/mrm/status` (below).
- The footprint comes from the vehicle model's `vehicle_info.param.yaml` (`mrm_core.launch.xml` argument `vehicle_model`).
- Runs at 20 Hz on the latest ego pose, so the first trajectory point is the ego rear axle in every cycle (MRM-001), also in standby.
- Geometry: Autoware's planned path (lane following, avoidance, lane change) with a cubic lateral offset from the ego pose back onto that path (MRM-002), sized for `blend_lat_acc_mps2` (0.5). At 8 m/s, 0.8 m offset plus 4 deg heading error need about 49 m, which is longer than the stop distance, so the car may stop before it is back on Autoware's path.
- Speed: only the jerk-limited stop profile (`decel_mps2` 1.0, `jerk_mps3` 1.0). **Autoware's planned speeds are not used**: in hot standby Autoware does not control the car, so its speed at the ego lags or leads the real speed (it replans only if the deviation is more than 5.5 m/s). A first version that used them as a limit produced a deceleration burst and speed bumps in the first meters.
- Autoware's **stop points** are used (a plan point <= `plan_stop_speed_mps` that is at least `plan_stop_min_dist_m` ahead, e.g. an obstacle). If the plan stops before the comfortable stop, the planner stops there with up to `plan_max_decel_mps2` (2.5) and `plan_max_jerk_mps3` (1.5), Autoware's absolute limits. With 1.0 m/s3 a stop from 8 m/s needs 22.7 m even at 2.5 m/s2, with 1.5 m/s3 19.5 m. If even that is not enough, the status says so and the maximum is used.
- The stop point itself is always a trajectory sample, and speeds never rise along the trajectory.
- On the trigger the stop profile and the stop point are latched, so the stop point does not creep while braking. A later obstacle may only move the stop point closer.
- Standby with an invalid input (no ego state, no plan, plan older than `max_input_age_s`, ego more than `max_offset_m` off the path): nothing is published (MRM-004). After the trigger the last plan is used.
- Obstacle: with the team's avoidance parameters, Autoware shifts its plan around a parked car that fits inside the lane (0.42 m in the sample map, lanes there are only about 3 m wide) and stops in front of one that does not fit. The MRM follows whatever Autoware plans.
- Scenario results (simulator, sample vehicle, steady 8 m/s): planned peak deceleration 1.005 m/s2, measured 1.13 m/s2 and jerk 1.1 m/s3. If the trigger comes while the nominal still accelerates (+1 m/s2), the measured peak is 1.6 m/s2 and 3.7 m/s3: the controller sees a step from +1 to -1 m/s2 at the switch. Starting the profile at the current acceleration instead of 0 would soften that (open decision).
- **Objects (MRM-008):** the ego footprint at every trajectory pose up to the stop point is checked against the perceived objects that move slower than `object_static_speed_mps` (1.0, the team's moving threshold; faster ones are left to Autoware's plan: obstacle stop / cruise). Conflict = closer than `object_margin_m` (0.1). An object that is already closer than the margin at the ego pose (the nominal passes it narrowly) only counts if the path gets more than `object_close_tolerance_m` (0.05) closer, so a narrow pass does not cause a hard stop by itself. On a conflict: first a longer blend is tried (`blend_extend_factors` 1.5, 2, 3, 5 x the default length): it stays on the ego's side longer and joins Autoware's path later. This adds no own lateral logic, it is only a slower return with less lateral acceleration. If every blend conflicts, the planner stops `object_stop_margin_m` (1.0) before the first conflicting pose with up to 2.5 m/s2 / 1.5 m/s3, as for Autoware's stop points. Cost: about 5 ms per cycle in the worst case (40 objects, 14 m/s, all blends checked).

## `/mrm/status`
`diagnostic_msgs/DiagnosticStatus`, name `mrm_stop_planner`, 20 Hz. `message` = `<state>` or `<state>: <warnings>`.

| State | Meaning | Level |
|---|---|---|
| `standby` | not triggered, a valid trajectory is published every cycle | OK, WARN with warnings |
| `active` | triggered, braking along the latched stop | OK / WARN |
| `stopped` | triggered, speed < 0.05 m/s and less than 0.5 m to the stop point | OK / WARN |
| `invalid` | no trajectory published this cycle (in standby: MRM-004); the reason is the message | ERROR |

Warnings (key `warnings`, comma separated). The trajectory is still published:

| Warning | Meaning |
|---|---|
| `path_short` | Autoware's path ends before the stop point; the trajectory stops at its end (MRM-007) |
| `plan_stop_infeasible` | Autoware stops closer than 2.5 m/s2 / 1.5 m/s3 allow; the maximum is used |
| `object_blend_extended` | a longer blend keeps the footprint away from an object |
| `object_stop` | every blend meets an object: stops before it (with more than the comfortable deceleration if needed) |
| `object_stop_infeasible` | ... and cannot stop before it; the maximum is used |
| `no_objects` | no object list for `max_objects_age_s` (1.0 s): the path is not checked against objects |
| `stale_plan` | triggered and Autoware's trajectory is older than `max_input_age_s`: the last one is still used |

Other keys: `state`, `triggered`, `plan_age_s`, `offset_m`, `heading_err_rad`, `blend_len_m`, `stop_distance_m`, `path_remaining_m`, `max_curvature`, and when they apply: `plan_stop_m`, `decel_used`, `blend_factor`, `object_conflict_m`.

## MRM trigger (`mrm_trigger_udp`)
UDP port 45000 (`trigger_port`). The packet format is a **placeholder** until the MABX team defines it: 1 byte, nonzero = trigger (`mrm_trigger_udp/packet.py`). The first trigger is latched by all nodes. A `False` or a lost link never undoes it or triggers.
