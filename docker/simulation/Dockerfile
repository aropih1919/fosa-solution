FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-colcon-common-extensions python3-rosdep python3-pytest \
       python3-opencv python3-yaml python3-numpy git \
       python3-pip curl unzip zip procps util-linux \
       ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-slam-toolbox \
       ros-jazzy-laser-filters ros-jazzy-depthimage-to-laserscan \
       ros-jazzy-audio-common-msgs ros-jazzy-rmw-cyclonedds-cpp \
       ros-jazzy-robot-localization ros-jazzy-ros2-control \
       ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
       ros-jazzy-image-transport-plugins ros-jazzy-topic-tools \
       ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro \
       ros-jazzy-rviz2 ros-jazzy-cv-bridge \
       ros-jazzy-tf2-ros-py ros-jazzy-sensor-msgs-py \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy
WORKDIR /workspace
CMD ["bash"]
