# Start from the official Autoware Jazzy CUDA image
FROM ghcr.io/autowarefoundation/autoware:universe-devel-cuda-jazzy

# Set shell to bash for better compatibility
SHELL ["/bin/bash", "-c"]

# Set working directory
WORKDIR /home/aw/autoware

# Upgrade base packages and refresh rosdep cache (cached across src/ changes)
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get upgrade -y && \
    rosdep update && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install workspace dependencies from source
RUN --mount=type=bind,source=src,target=/home/aw/autoware/src \
    DEBIAN_FRONTEND=noninteractive apt-get update && \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    DEBIAN_FRONTEND=noninteractive rosdep install -y \
        --from-paths src \
        --ignore-src \
        --rosdistro "${ROS_DISTRO}" \
        --skip-keys "autoware_minimap_overlay_rviz_plugin autoware_polar_grid common_sensor_launch tier4_datetime_rviz_plugin" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# domain_bridge: copies the sensor topics from the nominal system's domain into the MRM domain (mrm_ioniq5.launch.xml)
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-domain-bridge && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# tmux + catmux: one container in the background, any number of terminals attach to its tmux session
# (docker exec -it -u aw <name> tmux new -A -s aw); catmux starts a whole window layout from config/catmux/*.yaml
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends tmux && \
    pip3 install --no-cache-dir --break-system-packages catmux==0.3.6 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
COPY docker/tmux.conf /etc/tmux.conf

# Global environment variables for all users and processes
ENV ROS_DOMAIN_ID=0 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    CYCLONEDDS_URI=file:///home/aw/autoware/config/cyclonedds.xml

# Create shared setup script and hook into interactive shells (including dynamic HOST_UID)
RUN printf '%s\n' \
        'source /opt/ros/jazzy/setup.bash' \
        'if [ -f /home/aw/autoware/install/setup.bash ]; then' \
        '    source /home/aw/autoware/install/setup.bash' \
        'fi' \
        > /etc/autoware_env.sh && \
    chmod 644 /etc/autoware_env.sh && \
    echo 'source /etc/autoware_env.sh' >> /etc/bash.bashrc && \
    echo 'source /etc/autoware_env.sh' >> /etc/skel/.bashrc

# Load automatically in non-interactive "bash -c" commands
ENV BASH_ENV=/etc/autoware_env.sh

# Wrapper entrypoint: grants the unprivileged user access to /dev/dri (needed for RViz rendering)
COPY --chmod=755 docker/custom-entrypoint.sh /custom-entrypoint.sh
ENTRYPOINT ["/custom-entrypoint.sh"]
CMD ["/bin/bash"]
