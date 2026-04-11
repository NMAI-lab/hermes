#!/bin/bash

# =============================
# Usage:
#   ./hermes.sh compile
#   ./hermes.sh run
#   ./hermes.sh all
#   ./hermes.sh -> default = all
# =============================

MODE=${1:-all}

compile() {
    echo "Compiling hermes..."
    docker build --build-arg ARCH=$(dpkg --print-architecture) -t hermes .
}

run() {
    echo "Running hermes..."
    docker rm hermes
    xhost +local:docker
    docker run --name hermes  -it \
        --env DISPLAY=$DISPLAY \
        --env QT_X11_NO_MITSHM=1 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        hermes
}

case "$MODE" in
    compile)
        compile
        ;;
    run)
        run
        ;;
    all)
        compile
        run
        ;;
    *)
        echo "Usage: $0 {compile|run|all}"
        exit 1
        ;;
esac

