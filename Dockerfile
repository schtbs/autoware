# Start from the official Autoware Humble CUDA image
FROM ghcr.io/autowarefoundation/autoware:universe-devel-cuda-jazzy

# Set shell to bash for better compatibility
SHELL ["/bin/bash", "-c"]

# Set working directory
WORKDIR /home/aw/autoware

#RUN pip install https://github.com/gezp/carla_ros/releases/download/carla-0.9.15-ubuntu-22.04/carla-0.9.15-cp310-cp310-linux_x86_64.whl

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