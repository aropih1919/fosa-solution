FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-colcon-common-extensions python3-rosdep python3-pytest \
       python3-opencv python3-yaml python3-numpy git \
       ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy
WORKDIR /workspace
CMD ["bash"]
