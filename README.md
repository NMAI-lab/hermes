# Hermes

<p align="center">
    <img src="hermes_create_description/rviz/hermes.png" alt="hermes" width="200">
</p>

Hermes is a simulator for a mobile robot, acting as a BDI agent, that navigates a custom maze through the use of a LiDAR sensor and various Bluetooth beacons placed at each intersection. 

## Description
Hermes has the ability to perform the following:
- Maintain a consistent distance with the wall.
- Navigate the maze with its many intersections and complete a full trip from point A to point B using its preloaded map of the beacon connections.
- Handle possible collisions and by pass obstacles through the use of its bumper sensor.

This robot is based on the [iRobot Create 3](https://edu.irobot.com/what-we-offer/create3) developed by the iRobot company.

In addition to that, Hermes uses the popular AgentSpeak language [Jason](https://jason-lang.github.io/) to implement the BDI architecture.

## Installation
1. Install the proper version of [ROS](https://docs.ros.org/en/foxy/Installation.html) on your device (I used ROS Foxy for mine)

2. Make sure to source your installation:
```
$ source /opt/ros/foxy/setup.bash
```

3. Create a ROS workspace for your system. Such as:
```
$ mkdir -p ~/hermes_ws/src
```

4. Install the appropriate version of [iRobot Create 3 Sim](https://github.com/iRobotEducation/create3_sim/tree/foxy).


5. Install the ROS dependencies:
```
$ cd ~/hermes_ws
$ rosdep install --from-path src -yi
```
6. Build all the ROS packages by doing:
```
$ colcon build --symlink-install
Starting >>> control_msgs
Starting >>> ros2_control_test_assets
Starting >>> controller_manager_msgs
Starting >>> realtime_tools
Starting >>> irobot_create_msgs
Finished <<< ros2_control_test_assets [0.11s]                     
Finished <<< realtime_tools [0.24s]                                   
Finished <<< controller_manager_msgs [0.46s]                          
Finished <<< irobot_create_msgs [0.47s]
Starting >>> irobot_create_gazebo_plugins
Finished <<< control_msgs [0.53s]                                   
Starting >>> hardware_interface
Starting >>> irobot_create_toolbox
Finished <<< irobot_create_gazebo_plugins [0.29s]
Finished <<< hardware_interface [0.29s]                                   
Starting >>> controller_interface
Starting >>> transmission_interface
Finished <<< irobot_create_toolbox [0.31s]
Finished <<< transmission_interface [0.15s]                                   
Finished <<< controller_interface [0.17s]
Starting >>> controller_manager
Finished <<< controller_manager [0.19s]                      
Starting >>> forward_command_controller
Starting >>> joint_state_broadcaster
Starting >>> diff_drive_controller
Starting >>> force_torque_sensor_broadcaster                               
Starting >>> imu_sensor_broadcaster
Finished <<< forward_command_controller [0.23s]
Starting >>> joint_trajectory_controller
Finished <<< imu_sensor_broadcaster [0.25s]
Finished <<< joint_state_broadcaster [0.27s]
Starting >>> tricycle_controller
Starting >>> effort_controllers
Finished <<< force_torque_sensor_broadcaster [0.28s]
Starting >>> joint_state_controller
Finished <<< diff_drive_controller [0.31s]                                    
Starting >>> position_controllers
Finished <<< joint_state_controller [0.21s]
Starting >>> velocity_controllers
Finished <<< joint_trajectory_controller [0.30s]
Starting >>> gazebo_ros2_control
Finished <<< effort_controllers [0.29s]
Starting >>> ros2controlcli
Finished <<< tricycle_controller [0.30s]
Starting >>> gripper_controllers
Finished <<< position_controllers [0.28s]
Finished <<< velocity_controllers [0.26s]                                  
Starting >>> ros2_controllers
Finished <<< gazebo_ros2_control [0.24s]
Finished <<< gripper_controllers [0.29s]                       
Finished <<< ros2_controllers [0.14s]
Starting >>> irobot_create_control
Finished <<< irobot_create_control [0.10s]                     
Starting >>> irobot_create_description
Starting >>> hermes_create_description                         
Finished <<< irobot_create_description [0.12s]                 
Starting >>> irobot_create_gazebo
Finished <<< hermes_create_description [0.13s]
Starting >>> hermes_environment
Finished <<< ros2controlcli [0.67s]                            
Starting >>> ros2_control
Finished <<< hermes_environment [0.13s]
Finished <<< irobot_create_gazebo [0.15s]
Starting >>> irobot_create_sim
Finished <<< ros2_control [0.10s]                                    
Starting >>> gazebo_ros2_control_demos
Finished <<< irobot_create_sim [0.13s]
Finished <<< gazebo_ros2_control_demos [0.16s]                  

Summary: 34 packages finished [2.82s]
```

7. Source your installation by doing:
```
$ source ~/hermes_ws/install/local_setup.bash
```

## Running Hermes

First try to fire up the simulator by doing:
```
$ ros2 launch hermes_environment environment.launch.py
```

You should see the Gazebo and RViz windows pop up:
<p align="center">
    <img src="demos/rviz_gazebo_demo.png" alt="hermes" width=500">
</p>

In a separate window try controlling the robot by running a few ROS commands:

- To move the robot in a separate terminal try:
```
$ source ~/hermes_ws/install/local_setup.bash
$ ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

- To dock the robot in a separate terminal try:
```
$ source ~/hermes_ws/install/local_setup.bash
$ ros2 action send_goal /dock irobot_create_msgs/action/DockServo "{}"
```

- To undock the robot in a separate terminal try:
```
$ source ~/hermes_ws/install/local_setup.bash
$ ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```