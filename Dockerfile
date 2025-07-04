# syntax=docker/dockerfile:1.3
FROM ubuntu:20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy

# Install basic tools and locales
RUN apt update && apt install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    git \
    sudo \
    python3-pip \
    wget \
    unzip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add GitHub's SSH key to known_hosts to avoid interactive trust prompt
RUN mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Add ROS 2 GPG key and repo
RUN curl -sSL http://repo.ros2.org/repos.key | apt-key add - && \
    add-apt-repository universe && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2-latest.list

# Install ROS 2 Foxy base packages
RUN apt update && apt install -y \
    ros-foxy-desktop \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-rviz2 \
    ros-foxy-ackermann-msgs \
    ros-foxy-joint-state-publisher \
    ros-foxy-control-toolbox \
    ros-foxy-xacro \
    default-jdk \
    gradle \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "IGNITION_VERSION=fortress" >> ~/.bashrc

# Create workspace
RUN mkdir -p /root/hermes_ws/src
WORKDIR /root/hermes_ws/src

# Clone Hermes
RUN --mount=type=ssh git clone git@github.com:bardia-p/hermes.git

# Install Python dependencies
RUN pip3 install --upgrade pip && pip3 install -r /root/hermes_ws/src/hermes/requirements.txt

# Import dependencies
RUN vcs import /root/hermes_ws/src/ < /root/hermes_ws/src/hermes/simulator_dependencies.repos

# Install rosdep and initialize
RUN apt update && apt install -y python3-rosdep \
    && source /opt/ros/foxy/setup.bash \
    && sudo rosdep init \
    && rosdep update

# Install ROS 2 dependencies
WORKDIR /root/hermes_ws
RUN source /opt/ros/foxy/setup.bash && \
    rosdep install --from-paths src -yi --skip-keys "ament_tools"

# Build and install Java gradle plugin
WORKDIR /root/hermes_ws/src/ros2-java/ament_gradle_plugin
RUN gradle uploadArchives

# Build ROS 2 workspace
WORKDIR /root/hermes_ws
RUN source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install

# Source workspace
RUN echo "source /root/hermes_ws/install/local_setup.bash" >> ~/.bashrc

# Default shell
CMD ["bash"]