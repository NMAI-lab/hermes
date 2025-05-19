# Hermes

<p align="center">
    <img src="images/hermes.png" alt="hermes" width="200">
    <br>
    <em>Logo designed by Daniel Pacada</em>
</p>

Hermes is a simulator for a mobile robot, acting as a BDI agent, that navigates a custom maze through the use of a LiDAR sensor and various Bluetooth beacons placed at each intersection. 

## Description
Hermes has the ability to perform the following:
- Maintain a consistent distance from the wall.
- Navigate the maze with its many intersections and complete a full trip from point A to point B using its preloaded map of the beacon connections.
- Handle possible collisions and by pass obstacles through the use of its bumper sensor.

This robot is based on the [iRobot Create 3](https://edu.irobot.com/what-we-offer/create3) developed by the iRobot company.

In addition to that, Hermes uses the popular AgentSpeak language [Jason](https://jason-lang.github.io/) to implement the BDI architecture.

## Installation
1. Install [ROS Foxy](https://docs.ros.org/en/foxy/Installation.html) on an Ubuntu 20.04 system

2. Make sure to source your installation:
```
source /opt/ros/foxy/setup.bash
```

3. Install the Open-JDK:
```
sudo apt install default-jdk
```

4. Install gradle:
```
sudo apt install gradle
```

5. Install ROS [Gazebo 11](https://classic.gazebosim.org/tutorials?tut=ros2_installing):
```
sudo apt install ros-foxy-gazebo-ros-pkgs
```

6. Install [RViz2](https://github.com/ros2/rviz):
```
sudo apt install ros-foxy-rviz2
```

7. Install all the required Python packages using:
```
pip install -r requirements.txt
```

8. Create a ROS workspace for your system. Such as:
```
mkdir -p ~/hermes_ws/src
cd ~/hermes_ws/src
```

9. Clone [hermes](https://github.com/bardia-p/hermes):
```
git clone git@github.com:bardia-p/hermes.git
```

10. Clone the appropriate ROS dependencies:
```
vcs import ~/hermes_ws/src/ < ~/hermes_ws/src/hermes/dependencies.repos
```

11. Install the ROS dependencies:
```
cd ~/hermes_ws
sudo rosdep init
rosdep update
rosdep install --from-path src -yi --skip-keys "ament_tools"
```

12. Install the ROS2 ament Java gradle plugin:
```
cd src/ros2-java/ament_gradle_plugin
gradle uploadArchives
```

13. Build all the ROS packages by doing:
```
colcon build --symlink-install
```

14. Source your installation by doing:
```
source ~/hermes_ws/install/local_setup.bash
```

**NOTE:** If at any point you face any issues with the installation process of these ROS dependencies, please refer to the README files of the appropriate repositories:
- [create3_sim](https://github.com/iRobotEducation/create3_sim/tree/foxy)
- [irobot_create_msgs](https://github.com/iRobotEducation/irobot_create_msgs)
- [ros2_java](https://github.com/ros2-java/ros2_java)

## Running Hermes

First try to fire up the simulator by doing:
```
source ~/hermes_ws/install/local_setup.bash
ros2 launch hermes_simulator simulator.launch.py start:=I1 end:=B1
```

You should see the Gazebo and RViz windows pop up:
<p align="center">
    <img src="images/rviz_gazebo_demo.png" alt="hermes" width=500">
</p>

In a separate window try controlling the robot by running a few ROS commands:

- **Moving the robot:** in a separate terminal try:
```
source ~/hermes_ws/install/local_setup.bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

- **Docking the robot:** in a separate terminal try:
```
source ~/hermes_ws/install/local_setup.bash
ros2 action send_goal /dock irobot_create_msgs/action/DockServo "{}"
```

- **Undocking the robot:** in a separate terminal try:
```
source ~/hermes_ws/install/local_setup.bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

## Project Structure
- **hermes_create_description:** This package includes the Gazebo descriptions for the robotcs, sensors, and the dock station. It also includes the appropriate launch files for spawning these objects.
- **hermes_environment:** This package includes the implementation of the simulation environment with the various configs for loading the robot map.
- **hermes_agent:** This package includes the implementation of the agent which is in charge of parsing the agent brain files.
- **hermes_simulator:** This package includes the implementation of the simulator with the various sensors for interpreting the environment. This package invokes the environment, the agent, and the sensors.

## Notes
- The inspiration for this project came from another similar project I worked on. Make sure to check out [Carleton Mail Delivery Robot](https://github.com/bardia-p/carleton-mail-delivery-robot)!
- The name Hermes is a nod to the previous major AgentSpeak projects namely, [Jason](https://github.com/jason-lang/jason) and [Peleus](https://github.com/meneguzzi/Peleus). I chose the name Hermes since the main purpose of this robot is to deliver mail from one place to another. 

## Acknowledgements
- [iRobot's Create 3 Simulator](https://github.com/iRobotEducation/create3_sim/tree/foxy) for the main robot simulator.
- [Jason](https://github.com/jason-lang/jason) for the AgentSpeak implementation of the project.
- [ros2_java](https://github.com/ros2-java/ros2_java.git) for the ros2 and Jason integration.
- [Peleus](https://github.com/meneguzzi/Peleus) for connecting Jason to various planners.
- [ENHSP](https://gitlab.com/enricos83/ENHSP-Public.git) for creating the robot plans.

## TO-DO
- Add a simple Jason implementation for wall following
- Connect Peleus and the ENHSP planner for this behaviour
- Add state machines and subsumption
- Add intersection handling