ARG ROS_DISTRO=foxy 

FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO=foxy
ARG ARCH

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Install additional ROS packages
RUN apt update && apt install -y \ 
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-control-toolbox \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-rosbridge-server
    
# Install additional packages
RUN apt update && apt install -y \
    software-properties-common \
    netcat \
    python3-pip

# Install Java 21
RUN apt update && \
    add-apt-repository ppa:openjdk-r/ppa && \ 
    apt update && \ 
    apt install -y openjdk-21-jdk openjdk-21-jre
    
RUN echo "Architecture is $ARCH"

ENV JAVA_HOME=/usr/lib/jvm/java-21-openjdk-${ARCH}
ENV PATH=$JAVA_HOME/bin:$PATH

# Install rosdep and initialize
RUN apt install -y \
    python3-rosdep \
    && source /opt/ros/$ROS_DISTRO/setup.bash \ 
    && rosdep init || true \ 
    && rosdep update

# Clean up apt libraries
RUN rm -rf /var/lib/apt/lists/*

# Download and Install Gradle
WORKDIR /opt/gradle
RUN curl -L https://services.gradle.org/distributions/gradle-8.14.2-bin.zip -o gradle.zip
RUN unzip gradle.zip
ENV GRADLE_HOME=/opt/gradle/gradle-8.14.2
ENV PATH=$GRADLE_HOME/bin:$PATH

# Source ROS setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
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
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep install --from-paths src -yi --skip-keys "ament_tools"

# Build and install Java gradle plugin
WORKDIR /root/hermes_ws/src/ros2-java/ament_gradle_plugin
RUN gradle publishToMavenLocal

# Build ROS 2 workspace
WORKDIR /root/hermes_ws

# Build the core ROS packages
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install

# Copy Hermes
COPY hermes_agent src/hermes/hermes_agent
COPY hermes_create_description src/hermes/hermes_create_description
COPY hermes_environment src/hermes/hermes_environment
COPY hermes_simulator src/hermes/hermes_simulator

# Build the Hermes packages
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install \
        --packages-select hermes_create_description hermes_environment hermes_agent hermes_simulator

# ROS2-Java Hotfix
RUN sed -i "/^CLASSPATH=/d" ./install/hermes_agent/lib/hermes_agent/hermes_agent

# Source workspace
RUN echo "source /root/hermes_ws/install/local_setup.bash" >> ~/.bashrc

# Start the container
COPY load_container.sh /root/load_container.sh
RUN chmod +x /root/load_container.sh
ENTRYPOINT ["/bin/bash", "/root/load_container.sh"]

# Default shell
CMD ["bash"]