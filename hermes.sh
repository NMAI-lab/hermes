#!/bin/bash

START="B3"
END="B4"
DISPLAY_MAS="true"
LAUNCH_AGENT="true"

cleanup() {
    echo "Stopping containers..."
    docker stop hermes-sim >/dev/null 2>&1
    docker rm hermes-sim >/dev/null 2>&1
    docker stop hermes-robot >/dev/null 2>&1
    docker rm hermes-robot >/dev/null 2>&1
}

trap cleanup SIGINT SIGTERM EXIT

usage() {
    echo "Usage: $0 [-s START] [-e END] [-d true|false] [-a true|false] <target> <mode> [ROS_VERSION]"
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
    echo "  -s START        Start node (default: B3)"
    echo "  -e END          End node (default: B1)"
    echo "  -d true|false   Display MAS (default: true)"
    echo "  -a true|false   Launch agent (default: true)"
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
    echo "Compiling hermes simulator (Docker): ROS=${ros_version}"
    docker build \
        --build-arg ARCH=$(dpkg --print-architecture) \
        --build-arg ROS_DISTRO="${ros_version}" \
        -f Dockerfile.simulator \
        -t hermes-sim .
}

simulator_docker_run() {
    local start="$1"
    local end="$2"
    local display_mas="$3"
    local launch_agent="$4"

    echo "Running hermes simulator (Docker): start=$start end=$end display_mas=$display_mas launch_agent=$launch_agent"

    docker stop hermes-sim 2>/dev/null && docker rm hermes-sim 2>/dev/null

    xhost +local:docker
    docker run --name hermes-sim -it \
        --env DISPLAY="$DISPLAY" \
        --env QT_X11_NO_MITSHM=1 \
        --gpus all \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -p 9090:9090 \
        --env LAUNCH_MODE=simulator \
        --env START=$start \
        --env END=$end \
        --env DISPLAY_MAS=$display_mas \
        --env LAUNCH_AGENT=$launch_agent \
        hermes-sim
}

simulator_local_compile() {
    echo "Compiling hermes simulator (local)..."

    cd ~/hermes_ws || { echo "Error: workspace not found"; exit 1; }
    rm -rf build/hermes_agent install/hermes_agent

    colcon build --symlink-install \
        --packages-select hermes_robot_description hermes_environment hermes_agent hermes_simulator
    sed -i "/^CLASSPATH=/d" ./install/hermes_agent/lib/hermes_agent/hermes_agent

    echo "Simulator build complete."
}

simulator_local_run() {
    local start="$1"
    local end="$2"
    local display_mas="$3"
    local launch_agent="$4"

    echo "Running hermes simulator (local): start=$start end=$end display_mas=$display_mas launch_agent=$launch_agent"

    ./perform_cleanup.sh
    source "~/hermes_ws/install/local_setup.bash"

    ros2 launch hermes_simulator simulator.launch.py \
        start:="$start" \
        end:="$end" \
        display_mas:="$display_mas" \
        run_hermes_agent:="$launch_agent"
}

robot_docker_compile() {
    local ros_version="$1"
    
    echo "Compiling hermes robot (Docker): ROS=$ros_version"

    docker build \
        --build-arg ROS_DISTRO=${ros_version} \
        -f Dockerfile.robot \
        -t hermes-robot .
}

robot_docker_run() {
    local end="$1"
    local launch_agent="$2"

    echo "Running hermes robot (Docker): end=$end launch_agent=$launch_agent"

    docker stop hermes-robot 2>/dev/null && docker rm hermes-robot 2>/dev/null

    docker run --name hermes-robot -it \
        --network host \
        --privileged \
        --env LAUNCH_MODE=robot \
        --env END=$end \
        --env LAUNCH_AGENT=$launch_agent \
        hermes-robot
}

robot_local_compile() {
    echo "Compiling hermes robot (local)..."

    cd "~/hermes_ws" || { echo "Error: workspace not found"; exit 1; }
    rm -rf build/hermes_agent install/hermes_agent

    source /opt/ros/humble/setup.bash

    colcon build --symlink-install
    sed -i "/^CLASSPATH=/d" ./install/hermes_agent/lib/hermes_agent/hermes_agent

    echo "Robot build complete."
}

robot_local_run() {
    local end="$1"
    local launch_agent="$2"

    echo "Running hermes robot (local): end=$end launch_agent=$launch_agent"

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>l4tbr0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'

    source "~/hermes_ws/install/local_setup.bash"

    ros2 launch hermes_simulator robot.launch.py \
        end:="$end" \
        run_hermes_agent:="$launch_agent"
}

while getopts "s:e:d:a:" opt; do
    case $opt in
        s) START="$OPTARG" ;;
        e) END="$OPTARG" ;;
        d) DISPLAY_MAS="$OPTARG" ;;
        a) LAUNCH_AGENT="$OPTARG" ;;
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
                simulator_docker_run "$START" "$END" "$DISPLAY_MAS" "$LAUNCH_AGENT"
                ;;
            docker-compile)
                simulator_docker_compile "$ROS_VERSION"
                ;;
            docker-run)
                simulator_docker_run "$START" "$END" "$DISPLAY_MAS" "$LAUNCH_AGENT"
                ;;
            local)
                simulator_local_compile
                simulator_local_run "$START" "$END" "$DISPLAY_MAS" "$LAUNCH_AGENT"
                ;;
            compile)
                simulator_local_compile
                ;;
            run)
                simulator_local_run "$START" "$END" "$DISPLAY_MAS" "$LAUNCH_AGENT"
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
                robot_docker_compile "$ROS_VERSION"
                robot_docker_run "$END" "$LAUNCH_AGENT"
                ;;
            docker-compile)
                robot_docker_compile "$ROS_VERSION"
                ;;
            docker-run)
                robot_docker_run "$END" "$LAUNCH_AGENT"
                ;;
            local)
                robot_local_compile
                robot_local_run "$END" "$LAUNCH_AGENT"
                ;;
            compile)
                robot_local_compile
                ;;
            run)
                robot_local_run "$END" "$LAUNCH_AGENT"
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
