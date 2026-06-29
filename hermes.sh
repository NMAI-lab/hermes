#!/bin/bash

WORKSPACE="${HERMES_WS:-$HOME/hermes_ws}"
START="B3"
END="B1"
DISPLAY_MAS="true"

usage() {
    echo "Usage: $0 [-w WORKSPACE] [-s START] [-e END] [-d true|false] <target> <mode> [ROS_VERSION]"
    echo ""
    echo "Targets:"
    echo "  simulator       The Gazebo simulation environment (ROS Foxy)"
    echo "  robot           The physical iRobot Create 3 on Jetson Orin Nano (ROS Humble)"
    echo ""
    echo "Modes:"
    echo "  docker          Build Docker image and run it"
    echo "  docker-compile  Build Docker image only"
    echo "  docker-run      Run existing Docker image"
    echo "  local           Build locally with colcon and launch"
    echo "  compile         Build locally with colcon only"
    echo "  run             Launch locally only"
    echo ""
    echo "Options:"
    echo "  -w WORKSPACE    Local workspace path (default: ~/hermes_ws)"
    echo "  -s START        Start node (default: B3)"
    echo "  -e END          End node (default: B1)"
    echo "  -d true|false   Display MAS (default: true)"
    echo ""
    echo "Examples:"
    echo "  $0 simulator docker"
    echo "  $0 simulator docker-compile foxy"
    echo "  $0 simulator local"
    echo "  $0 robot docker-compile"
    echo "  $0 robot docker-run"
    echo "  $0 robot local"
    echo "  $0 robot compile"
    echo "  $0 robot run"
    exit 1
}

simulator_docker_compile() {
    local ros_version="$1"
    echo "Compiling hermes simulator (Docker, ROS ${ros_version})..."
    docker build \
        --build-arg ARCH=$(dpkg --print-architecture) \
        --build-arg ROS_DISTRO="${ros_version}" \
        -f Dockerfile.simulator \
        -t hermes-sim .
}

simulator_docker_run() {
    echo "Running hermes simulator (Docker)..."
    docker rm hermes-sim 2>/dev/null
    xhost +local:docker
    docker run --name hermes-sim -it \
        --env DISPLAY="$DISPLAY" \
        --env QT_X11_NO_MITSHM=1 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -p 9090:9090 \
        hermes-sim
}

simulator_local_compile() {
    echo "Compiling hermes simulator (local)..."
    cd "$WORKSPACE" || { echo "Error: workspace not found: $WORKSPACE"; exit 1; }
    rm -rf build/hermes_agent install/hermes_agent
    colcon build --symlink-install \
        --packages-select hermes_create_description hermes_environment hermes_agent hermes_simulator
    sed -i "/^CLASSPATH=/d" ./install/hermes_agent/lib/hermes_agent/hermes_agent
    echo "Simulator build complete."
}

simulator_local_run() {
    echo "Running hermes simulator (local): start=$START end=$END display_mas=$DISPLAY_MAS"
    ./perform_cleanup.sh
    source "$WORKSPACE/install/local_setup.bash"
    ros2 launch hermes_simulator simulator.launch.py \
        start:="$START" \
        end:="$END" \
        display_mas:="$DISPLAY_MAS"
}

robot_docker_compile() {
    echo "Compiling hermes robot (Docker, ROS Humble)..."
    docker build \
        --build-arg ROS_DISTRO=humble \
        -f Dockerfile.robot \
        -t hermes-robot .
}

robot_docker_run() {
    echo "Running hermes robot (Docker)..."
    docker rm hermes-robot 2>/dev/null
    docker run --name hermes-robot -it \
        --network host \
        --privileged \
        -p 9090:9090 \
        hermes-robot
}

robot_local_compile() {
    echo "Compiling hermes robot (local)..."
    cd "$WORKSPACE" || { echo "Error: workspace not found: $WORKSPACE"; exit 1; }
    rm -rf build/hermes_agent install/hermes_agent
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install \
        --packages-select hermes_create_description hermes_environment hermes_agent
    sed -i "/^CLASSPATH=/d" ./install/hermes_agent/lib/hermes_agent/hermes_agent
    echo "Robot build complete."
}

robot_local_run() {
    echo "Running hermes robot (local)..."
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>usb0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
    source "$WORKSPACE/install/local_setup.bash"
    ros2 launch hermes_agent agent.launch.py
}

while getopts "w:s:e:d:" opt; do
    case $opt in
        w) WORKSPACE="$OPTARG" ;;
        s) START="$OPTARG" ;;
        e) END="$OPTARG" ;;
        d) DISPLAY_MAS="$OPTARG" ;;
        *) usage ;;
    esac
done
shift $((OPTIND - 1))

TARGET="${1}"
MODE="${2}"
ROS_VERSION="${3:-foxy}"

case "$TARGET" in

    simulator)
        case "$MODE" in
            docker)
                simulator_docker_compile "$ROS_VERSION"
                simulator_docker_run
                ;;
            docker-compile)
                simulator_docker_compile "$ROS_VERSION"
                ;;
            docker-run)
                simulator_docker_run
                ;;
            local)
                simulator_local_compile
                simulator_local_run
                ;;
            compile)
                simulator_local_compile
                ;;
            run)
                simulator_local_run
                ;;
            *)
                echo "Unknown mode for simulator: $MODE"
                usage
                ;;
        esac
        ;;

    robot)
        case "$MODE" in
            docker)
                robot_docker_compile
                robot_docker_run
                ;;
            docker-compile)
                robot_docker_compile
                ;;
            docker-run)
                robot_docker_run
                ;;
            local)
                robot_local_compile
                robot_local_run
                ;;
            compile)
                robot_local_compile
                ;;
            run)
                robot_local_run
                ;;
            *)
                echo "Unknown mode for robot: $MODE"
                usage
                ;;
        esac
        ;;

    *)
        echo "Unknown target: $TARGET"
        usage
        ;;
esac
