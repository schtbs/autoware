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
