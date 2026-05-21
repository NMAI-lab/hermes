#!/bin/bash

set -e

# Preparing the environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /root/hermes_ws/install/local_setup.bash

# Launch the ROS2 bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /root/ros_bridge.log 2>&1 &

# Wait for rosbridge to be ready before anything else tries to connect
echo "Waiting for rosbridge..."
until nc -z localhost 9090; do sleep 1; done
echo "ROS-Bridge is up"

# Keep container interactive
exec bash