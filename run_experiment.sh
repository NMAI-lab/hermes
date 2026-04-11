#!/bin/bash

if ! docker images | grep -q hermes; then
    echo "Hermes not found, building..."
    docker build --build-arg ARCH=$(dpkg --print-architecture) -t hermes .
else
    echo "Hermes image already exists, skipping build."
fi

TRIALS=(
    "start:=B4 end:=B3"
    "start:=B4 end:=B2"
    "start:=B4 end:=B1"
    "x:=-7 y:=-1 end:=B4"
)

METRIC_FILES=(
    "right_turn.json"
    "straight.json"
    "left_turn.json"
    "u_turn.json"
)

xhost +local:docker

for i in "${!TRIALS[@]}"; do
    echo "=== Running trial $((i+1)) ==="

    docker run --name hermes -d \
        --env DISPLAY=$DISPLAY \
        --env QT_X11_NO_MITSHM=1 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        hermes \
        /bin/bash -c "sleep infinity"

    timeout 180 docker exec hermes \
        /bin/bash -c "source /opt/ros/foxy/setup.bash && source /root/hermes_ws/install/local_setup.bash && ros2 launch hermes_simulator simulator.launch.py ${TRIALS[$i]} map:=experiment_map.json metrics_file:=${METRIC_FILES[$i]} rviz:=false gui:=false > /dev/null" &
    
    TRIAL_PID=$!

    start=$SECONDS
    while kill -0 $TRIAL_PID 2>/dev/null; do
        elapsed=$(( SECONDS - start ))
        printf "\r⏱  Trial $((i+1)) running... %02d:%02d" $(( elapsed/60 )) $(( elapsed%60 ))
        sleep 1
    done
    echo ""

    wait $TRIAL_PID

    if docker cp hermes:/root/hermes_ws/${METRIC_FILES[$i]} ./data_analysis/data/tmp.json 2>/dev/null; then
        echo "Copied ${METRIC_FILES[$i]}"

        # Merge into combined file
        if [ -f ./data_analysis/data/${METRIC_FILES[$i]} ]; then
            jq -s 'add' ./data_analysis/data/tmp.json ./data_analysis/data/${METRIC_FILES[$i]} > ./data_analysis/data/merged.json
            mv ./data_analysis/data/merged.json ./data_analysis/data/${METRIC_FILES[$i]}
            rm ./data_analysis/data/tmp.json
        else
            mv ./data_analysis/data/tmp.json ./data_analysis/data/${METRIC_FILES[$i]} 
        fi
    else
        echo "Failed to copy ${METRIC_FILES[$i]} — trial may have produced no data"
    fi

    echo "=== Trial $((i+1)) done ==="
    docker stop hermes && docker rm hermes
done