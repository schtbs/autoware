# MRM stack on the Ioniq 5

State: **prepared, not run on the car.** Everything here was checked in the planning simulator, with the Ioniq 5 vehicle model on the campus map, or in isolation. Nothing was tested with the real sensors, the MicroAutoBox or the nominal AD system.

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

```bash
export ROS_DOMAIN_ID=<MRM domain>      # not the domain of the nominal system
export CYCLONEDDS_URI=file:///home/aw/autoware/config/cyclonedds.xml
ros2 launch mrm_launch mrm_ioniq5.launch.xml map_path:=<map dir>   # e.g. .../maps/autoware_maps/campus_ost
```
It starts Autoware (`autoware_launch/autoware.launch.xml`, `ioniq5_vehicle` / `ioniq5_sensor_kit`), the MRM core, and the trigger receiver.
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

The nominal system is ROS 2 on the same network. Generic Autoware topics (`/planning/*`, `/control/*`, `/localization/*`, ...) must not meet. Use a separate `ROS_DOMAIN_ID` for the MRM stack. Only UDP crosses to the MicroAutoBox, nothing crosses by ROS.

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

0. That the launch argument `trajectory_input_topic` set in `mrm_ioniq5.launch.xml` really reaches the bridge (`autobox_trajectory_bridge`, `input_topic`). The fork change is trivial, but an isolated test did not start the bridge node here, so this is **unverified**. Check `ros2 param get /autobox_trajectory_bridge input_topic` first. `mrm_ioniq5.launch.xml` itself was only parsed, never started.

1. Latency chain planner to MicroAutoBox (MRM-004: age <= 0.1 s). The simulator only measures up to the ROS topic.
2. Difference between `/localization/kinematic_state` (EKF) and the raw A-ROX pose that the MABX uses (MRM-001, drive check).
3. Rear-axle reference of the pose (MRM-003, lever arm confirmed, drive check open).
4. Real trigger packet and the reaction of the MABX at standstill.
5. Load: Autoware perception plus the Python MRM nodes on the MRM PC at 20 Hz.
6. Built-in Autoware MRM handler: it still runs in the MRM Autoware (comfortable stop is enabled in the fork). It can change speed limits in the planning, which the MRM stack ignores (it does not use Autoware's planned speeds). It cannot reach the vehicle, because no Autoware command is sent to it. Not tested with a real fault.
7. Map: lanes of the real route (width, stop lines, traffic lights in `campus_ost`). The sample-map virtual traffic light stopped Autoware's plan in the simulator, such elements on the real map need a look.
