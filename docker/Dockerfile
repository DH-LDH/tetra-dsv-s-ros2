# ROS 2 Humble + Gazebo Fortress development image for the TETRA-DSV-S stack.
#
# This mirrors the Jetson Orin Nano deployment target (Ubuntu 22.04 / Humble) so
# nothing has to be ported twice. The host runs Jazzy + Harmonic, where the
# Gazebo plugin names differ (gz-sim-* instead of ignition-gazebo-*).
FROM osrf/ros:humble-desktop

ARG USERNAME=dev
ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive

# ros-gz on Humble binds to Fortress, which is what the reference Scout
# workspace's plugin names expect.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-gz \
      ros-humble-ros-gz-bridge \
      ros-humble-ros-gz-sim \
      ros-humble-slam-toolbox \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-pointcloud-to-laserscan \
      ros-humble-xacro \
      ros-humble-joint-state-publisher-gui \
      ros-humble-teleop-twist-keyboard \
      ros-humble-velodyne \
      ros-humble-robot-localization \
      ros-humble-tf2-tools \
      python3-colcon-common-extensions \
      python3-vcstool \
      mesa-utils \
      vim less \
    && rm -rf /var/lib/apt/lists/*

# Match the host UID/GID so files written into the mounted workspace stay owned
# by the host user instead of root.
RUN if getent group ${GID} >/dev/null; then \
        EXISTING=$(getent group ${GID} | cut -d: -f1); \
        groupmod -n ${USERNAME} "$EXISTING"; \
    else \
        groupadd -g ${GID} ${USERNAME}; \
    fi \
 && if getent passwd ${UID} >/dev/null; then \
        EXISTING=$(getent passwd ${UID} | cut -d: -f1); \
        usermod -l ${USERNAME} -d /home/${USERNAME} -m "$EXISTING"; \
    else \
        useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME}; \
    fi \
 && apt-get update && apt-get install -y --no-install-recommends sudo \
 && rm -rf /var/lib/apt/lists/* \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /ws

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
 && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash"]
