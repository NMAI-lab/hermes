#!/bin/bash

while getopts "f" opt; do
    case $opt in
        f) FORCE_COMPILE=true ;;
        *) echo "Usage: $0 [-f] [ROS_VERSION]"; exit 1 ;;
    esac
done
shift $((OPTIND - 1))
ROS_VERSION=${1:-foxy}

if [ "$FORCE_COMPILE" = true ]; then
    echo "Compiling hermes..."
    docker build --build-arg ARCH=$(dpkg --print-architecture) --build-arg ROS_DISTRO=${ROS_VERSION} -t hermes .
fi

echo "Running hermes..."
docker rm hermes 2>/dev/null
xhost +local:docker
docker run --name hermes  -it \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    hermes
