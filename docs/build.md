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
