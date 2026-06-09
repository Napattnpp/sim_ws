# 1. Use the official ROS Foxy base image
FROM ros:foxy

# 2. ROS core setup & locales
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8
ENV ROS_DOMAIN_ID=22

# 4. Additional installations
RUN apt-get update && apt-get install -y \ 
    nano \
    vim \
    tmux \
    git \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    libeigen3-dev \
    ros-foxy-rviz2 \
    ros-foxy-rosbridge-server \
    ros-foxy-turtlesim \
    ros-foxy-rqt \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-rqt-common-plugins \
    ros-foxy-slam-toolbox \
    ros-foxy-cartographer \
    ros-foxy-cartographer-ros \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-asio-cmake-module \
    ros-foxy-joy \
    ros-foxy-urg-node \
    ros-foxy-xacro \
    ros-foxy-robot-state-publisher \
    ros-foxy-joy-teleop \
    ros-foxy-ackermann-msgs \
    ros-foxy-diagnostic-updater \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || true \
    && rosdep update

RUN pip3 install transforms3d

# 5. Install f1tenth gym dependencies
RUN git clone https://github.com/f1tenth/f1tenth_gym /f1tenth_gym \
    && cd /f1tenth_gym \
    && pip3 install -e .

# 6. Configuring environment
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# 7. Entrypoint Setup
RUN echo '#!/bin/bash' > /ros_entrypoint.sh \
    && echo 'source /opt/ros/foxy/setup.bash' >> /ros_entrypoint.sh \
    && echo 'exec "$@"' >> /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

WORKDIR /sim_ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]