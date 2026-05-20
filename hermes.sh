#!/bin/bash

WORKSPACE="${HERMES_WS:-$HOME/hermes_ws}"
START="B3"
END="B1"
DISPLAY_MAS="true"

usage() {
    echo "Usage: $0 [-w WORKSPACE] [-s START] [-e END] [-d true|false] <mode> [ROS_VERSION]"
    echo ""
    echo "Modes:"
    echo "  docker          Build and run in Docker"
    echo "  docker-compile  Build Docker image only"
    echo "  docker-run      Run existing Docker image"
    echo "  local           Build and run locally"
    echo "  compile         Build locally with colcon"
    echo "  run             Launch ROS2 simulator locally"
    echo ""
    echo "Options:"
    echo "  -w WORKSPACE    Local workspace path (default: ~/hermes_ws)"
    echo "  -s START        Simulator start node (default: B3)"
    echo "  -e END          Simulator end node (default: B1)"
    echo "  -d true|false   Display MAS (default: true)"
    echo ""
    echo "ROS_VERSION defaults to 'foxy' (docker modes only)"
    exit 1
}

docker_compile() {
    local ros_version="$1"
    echo "Compiling hermes (Docker)..."
    docker build \
        --build-arg ARCH=$(dpkg --print-architecture) \
        --build-arg ROS_DISTRO="${ros_version}" \
        -t hermes .
}

docker_run() {
    echo "Running hermes (Docker)..."
    docker rm hermes 2>/dev/null
    xhost +local:docker
    docker run --name hermes -it \
        --env DISPLAY="$DISPLAY" \
        --env QT_X11_NO_MITSHM=1 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        hermes
}

local_compile() {
    echo "Compiling hermes (local)..."
    cd "$WORKSPACE" || { echo "Error: workspace not found: $WORKSPACE"; exit 1; }
    rm -rf build/hermes_agent install/hermes_agent
    colcon build --symlink-install
    sed -i "/^CLASSPATH=/d" ./install/hermes_agent/lib/hermes_agent/hermes_agent
    echo "Build complete."
}

local_run() {
    echo "Running hermes (local): start=$START end=$END display_mas=$DISPLAY_MAS"
    source $WORKSPACE/install/local_setup.bash
    ros2 launch hermes_simulator simulator.launch.py \
        start:="$START" \
        end:="$END" \
        display_mas:="$DISPLAY_MAS"
}

# Parse options
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

MODE="${1}"
ROS_VERSION="${2:-foxy}"

case "$MODE" in
    docker)
        docker_compile "$ROS_VERSION"
        docker_run
        ;;
    docker-compile)
        docker_compile "$ROS_VERSION"
        ;;
    docker-run)
        docker_run
        ;;
    local)
        local_compile
        local_run
        ;;
    compile)
        local_compile
        ;;
    run)
        local_run
        ;;
    *)
        echo "Unknown mode: $MODE"
        usage
        ;;
esac