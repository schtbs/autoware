# Start from the official Autoware Jazzy CUDA image
FROM ghcr.io/autowarefoundation/autoware:universe-devel-cuda-jazzy

# Set shell to bash for better compatibility
SHELL ["/bin/bash", "-c"]

# Set working directory
WORKDIR /home/aw/autoware

# Install CARLA Python client
# RUN pip install https://github.com/gezp/carla_ros/releases/download/carla-0.9.15-ubuntu-22.04/carla-0.9.15-cp310-cp310-linux_x86_64.whl

# Update rosdep and install dependencies from source
RUN --mount=type=bind,source=src,target=/home/aw/autoware/src \
    sudo apt-get update && \
    sudo apt-get upgrade -y && \
    rosdep update && \
    sudo rosdep install -y \
        --from-paths src \
        --ignore-src \
        --rosdistro $ROS_DISTRO && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Global environment variables for all users and processes
ENV ROS_DOMAIN_ID=0 \
    ROS_LOCALHOST_ONLY=0 \
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