#!/bin/bash

set -e

# Preparing the environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /root/hermes_ws/install/local_setup.bash

# Launch the ROS2 bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /root/ros_bridge.log 2>&1 &

ROSBRIDGE_PID=$!

# Wait for rosbridge to be ready before anything else tries to connect
echo "Waiting for rosbridge..."
until nc -z localhost 9090; do sleep 1; done
echo "ROS-Bridge is up"

if [[ -t 0 ]]; then
    case "${LAUNCH_MODE}" in

    simulator)
        echo "Starting simulator..."
        exec ros2 launch hermes_simulator simulator.launch.py \
            start:=${START:-B3} \
            end:=${END:-B1} \
            display_mas:=${DISPLAY_MAS:-true} \
            run_hermes_agent:=${LAUNCH_AGENT:-true}
        ;;

    robot)
        echo "Starting robot agent..."
        exec ros2 launch hermes_simulator robot.launch.py \
            end:=${END:-B1} \
            run_hermes_agent:=${LAUNCH_AGENT:-true}
        ;;

    *)
        echo "Unknown LAUNCH_MODE: ${LAUNCH_MODE}"
        exit 1
        ;;
    esac

else
    wait $ROSBRIDGE_PID
fi