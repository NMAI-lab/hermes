#!/bin/bash

while getopts "ft:" opt; do
    case $opt in
        f) FORCE_COMPILE=true ;;
        t) TIMEOUT=$OPTARG ;;
        *) echo "Usage: $0 [-f] [-t TIMEOUT] [TRIALS] [ROS_VERSION]"; exit 1 ;;
    esac
done
shift $((OPTIND - 1))
TRIALS=${1:-30}
ROS_VERSION=${2:-foxy}
TIMEOUT=${TIMEOUT:-60}

EXPERIMENTS=(
    "x:=-7 y:=-1 end:=B3"
    "x:=-7 y:=-1 end:=B2"
    "x:=-7 y:=-1 end:=B1"
    "x:=-7 y:=-1 end:=B4"
    "x:=-7 y:=1 yaw:=3.14159 end:=B4"
    "x:=-5 y:=-1 yaw:=3.14159 end:=B3"
)

METRIC_FILES=(
    "right_turn_simulator.json"
    "pass_through_simulator.json"
    "left_turn_simulator.json"
    "u_turn_simulator.json"
    "docking_simulator.json"
    "collision_handling_simulator.json"
)

cleanup() {
    echo "Caught SIGTERM — cleaning up..."

    docker stop hermes-sim && docker rm hermes-sim

    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

if ! docker images | grep -q hermes || [ "$FORCE_COMPILE" = true ]; then
    if [ "$FORCE_COMPILE" = true ]; then
        echo "Force compile requested, rebuilding..."
        docker rmi hermes 2>/dev/null
    else
        echo "Hermes not found, building..."
    fi

    docker build \
        --build-arg ARCH=$(dpkg --print-architecture) \
        --build-arg ROS_DISTRO="${ROS_VERSION}" \
        -f Dockerfile.simulator \
        -t hermes-sim .
else
    echo "Hermes image already exists, skipping build."
fi

docker stop hermes-sim && docker rm hermes-sim

for i in "${!EXPERIMENTS[@]}"; do
    echo "=== Running Experiment ${EXPERIMENTS[$i]} ==="

    for trial in $(seq 1 $TRIALS); do
        echo "=== Running Trial ${trial}/${TRIALS} ==="

        docker run --name hermes-sim -d \
            --env DISPLAY=$DISPLAY \
            --env QT_X11_NO_MITSHM=1 \
            --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
            hermes-sim \
            /bin/bash -c "sleep infinity"

        timeout ${TIMEOUT} docker exec hermes-sim /bin/bash -lc "
            source /opt/ros/${ROS_VERSION}/setup.bash &&
            source /root/hermes_ws/install/local_setup.bash &&
            ros2 launch hermes_simulator simulator.launch.py \
                ${EXPERIMENTS[$i]} \
                map:=experiment_map.json \
                metrics_file:=${METRIC_FILES[$i]} \
                rviz:=false \
                gui:=false \
            > /dev/null
        " &
        
        TRIAL_PID=$!

        start=$SECONDS
        while kill -0 $TRIAL_PID 2>/dev/null; do
            elapsed=$(( SECONDS - start ))
            printf "\r⏱  Experiment $((i+1))/${#EXPERIMENTS[@]}, Trial ${trial}/${TRIALS}... %02d:%02d" $(( elapsed/60 )) $(( elapsed%60 ))
            sleep 1
        done
        echo ""

        wait $TRIAL_PID

        if docker cp hermes-sim:/root/hermes_ws/${METRIC_FILES[$i]} ./miscellaneous/data_analysis/metrics_analysis/data/tmp.json 2>/dev/null; then
            echo "Copied ${METRIC_FILES[$i]}"

            if [ -f ./miscellaneous/data_analysis/metrics_analysis/data/${METRIC_FILES[$i]} ]; then
                head -c -2 ./miscellaneous/data_analysis/metrics_analysis/data/${METRIC_FILES[$i]} > ./miscellaneous/data_analysis/metrics_analysis/data/merged.json
                echo "," >> ./miscellaneous/data_analysis/metrics_analysis/data/merged.json
                tail -c +2 ./miscellaneous/data_analysis/metrics_analysis/data/tmp.json >> ./miscellaneous/data_analysis/metrics_analysis/data/merged.json
                mv ./miscellaneous/data_analysis/metrics_analysis/data/merged.json ./miscellaneous/data_analysis/metrics_analysis/data/${METRIC_FILES[$i]}
                rm ./miscellaneous/data_analysis/metrics_analysis/data/tmp.json
            else
                mv ./miscellaneous/data_analysis/metrics_analysis/data/tmp.json ./miscellaneous/data_analysis/metrics_analysis/data/${METRIC_FILES[$i]} 
            fi
        else
            echo "Failed to copy ${METRIC_FILES[$i]} — trial may have produced no data"
        fi

        echo "=== Trial ${trial}/${TRIALS} done ==="
        docker stop hermes-sim && docker rm hermes-sim
    done
done