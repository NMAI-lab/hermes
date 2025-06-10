#!/bin/bash

# Clean Gazebo cache, shared memory, and semaphores
rm -rf /tmp/gazebo* /tmp/launch_params* /dev/shm/fastrtps* /dev/shm/sem.fastrtps*

# Kill leftover Gazebo processes
pkill -f gzserver
pkill -f gzclient
