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
1. Install the proper version of [ROS](https://docs.ros.org/en/foxy/Installation.html) on the appropriate Debian system
(My setup is ROS Foxy on Ubuntu 20.04)

3. Make sure to source your installation:
```
$ source /opt/ros/${ROS_DISTRO}>/setup.bash
```

3. Install ROS [Gazebo](https://gazebosim.org/docs/latest/ros_installation/):
```
$ sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

4. Install [RViz2](https://github.com/ros2/rviz):
```
$ sudo apt install ros-${ROS_DISTRO}-rviz2
```

5. Install [colcon-common-extensions](https://pypi.org/project/colcon-common-extensions/):
```
$ pip install colcon-common-extensions
```

6. Install [rosdep](https://pypi.org/project/rosdep/):
```
$ sudo pip install rosdep
```

7. Install [vcs](https://pypi.org/project/vcstool/):
```
$ pip install vcstool
```

8. Create a ROS workspace for your system. Such as:
```
$ mkdir -p ~/hermes_ws/src
$ cd ~/hermes_ws/src
```

9. Clone the appropriate version of [iRobot Create 3 Sim](https://github.com/iRobotEducation/create3_sim):
```
$ git clone -b ${ROS_DISTRO} git@github.com:iRobotEducation/create3_sim.git
$ vcs import ~/hermes_ws/src/ < ~/hermes_ws/src/create3_sim/dependencies.repos
```

10. Clone [hermes](https://github.com/bardia-p/hermes):
```
$ git clone git@github.com:bardia-p/hermes.git
```

11. Install the ROS dependencies:
```
$ cd ~/hermes_ws
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-path src -yi
```

12. Build all the ROS packages by doing:
```
$ colcon build --symlink-install
```

13. Source your installation by doing:
```
$ source ~/hermes_ws/install/local_setup.bash
```

## Running Hermes

First try to fire up the simulator by doing:
```
$ source ~/hermes_ws/install/local_setup.bash
$ ros2 launch hermes_environment environment.launch.py
```

You should see the Gazebo and RViz windows pop up:
<p align="center">
    <img src="images/rviz_gazebo_demo.png" alt="hermes" width=500">
</p>

In a separate window try controlling the robot by running a few ROS commands:

- **Moving the robot:** in a separate terminal try:
```
$ source ~/hermes_ws/install/local_setup.bash
$ ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

- **Docking the robot:** in a separate terminal try:
```
$ source ~/hermes_ws/install/local_setup.bash
$ ros2 action send_goal /dock irobot_create_msgs/action/DockServo "{}"
```

- **Undocking the robot:** in a separate terminal try:
```
$ source ~/hermes_ws/install/local_setup.bash
$ ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

## Project Structure
- **hermes_create_description:** This package includes the Gazebo descriptions for the robotcs, sensors, and the dock station. It also includes the appropriate launch files for spawning these objects.
- **hermes_environment:** This package includes the implementation for the simulator environment with the various configs for loading the robot map.

## Notes
- The inspiration for this project came from another similar project I worked on. Make sure to check out [Carleton Mail Delivery Robot](https://github.com/bardia-p/carleton-mail-delivery-robot)!
- The name Hermes is a nod to the previous major AgentSpeak projects namely, [Jason](https://github.com/jason-lang/jason) and [Peleus](https://github.com/meneguzzi/Peleus). I chose the name Hermes since the main purpose of this robot is to deliver mail from one place to another. 

## TO-DO
- Add beacons to the simulator
- Add custom map generation
- Add a simple Jason implementation for the robot
