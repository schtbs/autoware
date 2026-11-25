# Build and test

Build and run only inside the Docker container (see `CLAUDE.md` for the commands).

```bash
source /opt/ros/jazzy/setup.bash
# whole workspace (slow, only when needed)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# our packages only (normal case)
colcon build --symlink-install --packages-select <pkg> [<pkg> ...]
source install/setup.bash
```

- Always use `--symlink-install` so Python nodes and launch/config files can be edited without rebuilding.
- Don't use `--packages-up-to` on top-level Autoware packages unless necessary, it triggers a huge rebuild.
- Tests: `colcon test --packages-select <pkg> && colcon test-result --verbose`.
- If the Dockerfile or the dependencies of `src/` change, rebuild the image.

## Tests

- Pure logic (no ROS needed, run on the host): `cd src/custom/autoware_custom/<package> && python3 -m pytest -q test`
  (`mrm_route_generator`, `mrm_stop_planner`, `mrm_sim_tools`). Run pytest from inside the package directory.
- Simulation scenarios with KPIs: `scripts/run_mrm_scenarios.sh` (see `CLAUDE.md`). Each scenario starts its own headless
  simulator in a container. Results: `build/mrm_results/<scenario>.txt` and `.json` (raw data).
- Pitfalls: with `docker run` pass `-e HOST_UID/HOST_GID` (the entrypoint switches to that user), do not use `-u`.
  Do not name a script `inspect.py` (it shadows the Python standard library).
