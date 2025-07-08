# syntax=docker/dockerfile:1.3
FROM ubuntu:20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy

SHELL ["/bin/bash", "-c"]

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
    ros-foxy-xacro

# Install Java 21
RUN apt install -y openjdk-21-jdk \
    openjdk-21-jre
ENV JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64
ENV PATH="$JAVA_HOME/bin:$PATH"


# Install rosdep and initialize
RUN apt install -y python3-rosdep \
    && source /opt/ros/foxy/setup.bash \
    && sudo rosdep init \
    && rosdep update

RUN rm -rf /var/lib/apt/lists/*

# Download and Install Gradle
WORKDIR /opt/gradle
RUN curl -L https://services.gradle.org/distributions/gradle-8.14.2-bin.zip -o gradle.zip
RUN unzip gradle.zip
ENV GRADLE_HOME=/opt/gradle/gradle-8.14.2
ENV PATH=$PATH:$GRADLE_HOME/bin

# Source ROS setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "IGNITION_VERSION=fortress" >> ~/.bashrc

# Create workspace
RUN mkdir -p /root/hermes_ws/src
WORKDIR /root/hermes_ws/src

# Install Python dependencies
COPY requirements.txt hermes/requirements.txt
RUN pip3 install -r /root/hermes_ws/src/hermes/requirements.txt

# Import dependencies
COPY simulator_dependencies.repos /root/hermes_ws/src/hermes/simulator_dependencies.repos
RUN vcs import /root/hermes_ws/src/ < /root/hermes_ws/src/hermes/simulator_dependencies.repos

# Install ROS 2 dependencies
WORKDIR /root/hermes_ws
RUN source /opt/ros/foxy/setup.bash && \
    rosdep install --from-paths src -yi --skip-keys "ament_tools"

# Build and install Java gradle plugin
WORKDIR /root/hermes_ws/src/ros2-java/ament_gradle_plugin
RUN gradle publishToMavenLocal

# Build ROS 2 workspace
WORKDIR /root/hermes_ws

# Copy Hermes
COPY hermes_agent src/hermes/hermes_agent
COPY hermes_create_description src/hermes/hermes_create_description
COPY hermes_environment src/hermes/hermes_environment
COPY hermes_simulator src/hermes/hermes_simulator

# Build the workspace
RUN source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install

# Hotfix
RUN sed -i "/^CLASSPATH=/d" ./install/hermes_agent/lib/hermes_agent/hermes_agent

# Source workspace
RUN echo "source /root/hermes_ws/install/local_setup.bash" >> ~/.bashrc

# Default shell
CMD ["bash"]