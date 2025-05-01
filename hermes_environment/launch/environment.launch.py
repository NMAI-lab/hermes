from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

import yaml
import os
import json
import math

def load_yaml_params(file_path):
    """Load parameters from a YAML file."""
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

world_header = """
<?xml version="1.0" encoding="utf-8"?>

<sdf version="1.6">
    <world name="default">
        <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>

        <!-- Sun -->
        <include>
            <uri>model://sun</uri>
        </include>

        <!-- Ground -->
        <model name="ground">
            <static>true</static>
            <pose>0 0 0 0 0 0</pose>
            <link name="ground_link">
                <collision name="ground_collision">
                    <geometry>
                        <plane>
                            <size>100 100</size>
                        </plane>
                    </geometry>
                </collision>
                <visual name="ground_visual">
                    <geometry>
                        <plane>
                            <size>100 100</size>
                        </plane>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/White</name>
                        </script>
                    </material>
                </visual>
            </link>
        </model>
"""

world_footer = """
    </world>
</sdf>
"""

beacon_template = """
        <!-- Beacon - {name} -->
        <model name="{name}">
            <static>true</static>
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name="body">
                <!-- Visual appearance of the RF system -->
                <visual name="beacon_visual">
                    <geometry>
                        <sphere>
                            <radius>0.1</radius>
                        </sphere>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/Red</name>
                        </script>
                    </material>
                </visual>
            </link>
        </model>
"""

wall_template = """
        <!-- Wall - {beacon}_{direction}_{side} -->
        <model name="wall_{beacon}_{direction}_{side}">
            <static>true</static>
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name="wall_{beacon}_{direction}_{side}_link">
                <collision name="wall_{beacon}_{direction}_{side}_collision">
                    <geometry>
                        <box>
                            <size>{width} {length} {height}</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="wall_{beacon}_{direction}_{side}_visual">
                    <geometry>
                        <box>
                            <size>{width} {length} {height}</size>
                        </box>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/Bricks</name>
                        </script>
                    </material>
                </visual>
            </link>
        </model>
"""

def generate_world(input_file, start_beacon, end_beacon, map_params):
    with open(input_file) as json_data:
        world_body = ""
        parsed_map = json.load(json_data)
        beacons_dicts = parsed_map['beacons']
        origin_beacon = parsed_map['origin']
        beacons = dict()
        robot_position = None
        dock_position = None
        
        to_visit = [(origin_beacon, 0, 0)]

        world_name = map_params['world_name']
        wall_height = map_params['wall_height']
        tunnel_width = map_params['tunnel_width']
        wall_thickness = map_params['wall_thickness']
        dock_offset = map_params['dock_offset']

        while len(to_visit) != 0:
            curr_beacon, x, y = to_visit.pop(0)

            world_body += beacon_template.format(name=curr_beacon, x=x, y=y, z=wall_height)

            curr_beacon_dict = beacons_dicts[curr_beacon]

            if curr_beacon_dict["type"] == "intersection":
                beacons[curr_beacon] = (x, y, 0, 0)
                if "north" in curr_beacon_dict:
                    distance = curr_beacon_dict["north"]["distance"]
                    world_body += wall_template.format(beacon=curr_beacon, direction="north", side="l", x=x-tunnel_width/2, y=y+tunnel_width/2+distance/4, z=wall_height//2, width=wall_thickness, length=distance/2, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="north", side="r", x=x+tunnel_width/2, y=y+tunnel_width/2+distance/4, z=wall_height//2,width=wall_thickness, length=distance/2, height=wall_height)
                    if curr_beacon_dict["north"]["name"] not in beacons:
                        to_visit.append((curr_beacon_dict["north"]["name"], x, y + curr_beacon_dict["north"]["distance"]))
                else:
                    world_body += wall_template.format(beacon=curr_beacon, direction="north", side="t", x=x, y=y+tunnel_width/2, z=wall_height//2, width=tunnel_width, length=wall_thickness, height=wall_height)

                if "east" in curr_beacon_dict:
                    world_body += wall_template.format(beacon=curr_beacon, direction="east", side="l", x=x+tunnel_width/2+distance/4, y=y-tunnel_width/2, z=wall_height//2, width=distance/2, length=wall_thickness, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="east", side="r", x=x+tunnel_width/2+distance/4, y=y+tunnel_width/2, z=wall_height//2,width=distance/2, length=wall_thickness, height=wall_height)
                    if curr_beacon_dict["east"]["name"] not in beacons:
                        to_visit.append((curr_beacon_dict["east"]["name"], x + curr_beacon_dict["east"]["distance"], y))
                else:
                    world_body += wall_template.format(beacon=curr_beacon, direction="east", side="t", x=x+tunnel_width/2, y=y, z=wall_height//2, width=wall_thickness, length=tunnel_width, height=wall_height)

                if "south" in curr_beacon_dict:
                    world_body += wall_template.format(beacon=curr_beacon, direction="south", side="l", x=x-tunnel_width/2, y=y-tunnel_width/2-distance/4, z=wall_height//2, width=wall_thickness, length=distance/2, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="south", side="r", x=x+tunnel_width/2, y=y-tunnel_width/2-distance/4, z=wall_height//2,width=wall_thickness, length=distance/2, height=wall_height)
                    if curr_beacon_dict["south"]["name"] not in beacons:
                        to_visit.append((curr_beacon_dict["south"]["name"], x, y - curr_beacon_dict["south"]["distance"]))
                else:
                    world_body += wall_template.format(beacon=curr_beacon, direction="south", side="t", x=x, y=y-tunnel_width/2, z=wall_height//2, width=tunnel_width, length=wall_thickness, height=wall_height)

                if "west" in curr_beacon_dict:
                    world_body += wall_template.format(beacon=curr_beacon, direction="west", side="l", x=x-tunnel_width/2-distance/4, y=y-tunnel_width/2, z=wall_height//2, width=distance/2, length=wall_thickness, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="west", side="r", x=x-tunnel_width/2-distance/4, y=y+tunnel_width/2, z=wall_height//2,width=distance/2, length=wall_thickness, height=wall_height)
                    if curr_beacon_dict["west"]["name"] not in beacons:
                        to_visit.append((curr_beacon_dict["west"]["name"], x - curr_beacon_dict["west"]["distance"], y))
                else:
                    world_body += wall_template.format(beacon=curr_beacon, direction="west", side="t", x=x-tunnel_width/2, y=y, z=wall_height//2, width=wall_thickness, length=tunnel_width, height=wall_height)
            elif curr_beacon_dict["type"] == "destination":
                if "north" in curr_beacon_dict:
                    beacons[curr_beacon] = (x, y+dock_offset, 0, math.pi/2)
                    distance = curr_beacon_dict["north"]["distance"]
                    world_body += wall_template.format(beacon=curr_beacon, direction="north", side="l", x=x-tunnel_width/2, y=y+distance/4, z=wall_height//2, width=wall_thickness, length=distance/2, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="north", side="r", x=x+tunnel_width/2, y=y+distance/4, z=wall_height//2,width=wall_thickness, length=distance/2, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="south", side="t", x=x, y=y, z=wall_height//2, width=tunnel_width, length=wall_thickness, height=wall_height)

                elif "east" in curr_beacon_dict:
                    beacons[curr_beacon] = (x+dock_offset, y, 0, 0)
                    world_body += wall_template.format(beacon=curr_beacon, direction="east", side="l", x=x+distance/4, y=y-tunnel_width/2, z=wall_height//2, width=distance/2, length=wall_thickness, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="east", side="r", x=x+distance/4, y=y+tunnel_width/2, z=wall_height//2,width=distance/2, length=wall_thickness, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="west", side="t", x=x, y=y, z=wall_height//2, width=wall_thickness, length=tunnel_width, height=wall_height)

                elif "south" in curr_beacon_dict:
                    beacons[curr_beacon] = (x, y-dock_offset, 0, -math.pi/2)
                    world_body += wall_template.format(beacon=curr_beacon, direction="south", side="l", x=x-tunnel_width/2, y=y-distance/4, z=wall_height//2, width=wall_thickness, length=distance/2, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="south", side="r", x=x+tunnel_width/2, y=y-distance/4, z=wall_height//2,width=wall_thickness, length=distance/2, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="north", side="t", x=x, y=y, z=wall_height//2, width=tunnel_width, length=wall_thickness, height=wall_height)

                elif "west" in curr_beacon_dict:
                    beacons[curr_beacon] = (x-dock_offset, y, 0, math.pi)
                    world_body += wall_template.format(beacon=curr_beacon, direction="west", side="l", x=x-distance/4, y=y-tunnel_width/2, z=wall_height//2, width=distance/2, length=wall_thickness, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="west", side="r", x=x-distance/4, y=y+tunnel_width/2, z=wall_height//2,width=distance/2, length=wall_thickness, height=wall_height)
                    world_body += wall_template.format(beacon=curr_beacon, direction="east", side="t", x=x, y=y, z=wall_height//2, width=wall_thickness, length=tunnel_width, height=wall_height)

        pkg_hermes_environment = get_package_share_directory('hermes_environment')
        hermes_world_filepath = os.path.join(
            pkg_hermes_environment,
            'worlds',
            world_name
        )
        output_file = open(hermes_world_filepath, "w")
        output_file.write(world_header + world_body + world_footer)
        output_file.close()

        if start_beacon in beacons:
            robot_position = (str(beacons[start_beacon][0]), str(beacons[start_beacon][1]), str(beacons[start_beacon][2]), str(beacons[start_beacon][3]))

        dock_position = (str(beacons[end_beacon][0]), str(beacons[end_beacon][1]), str(beacons[end_beacon][2]), str(beacons[end_beacon][3]))
        
        return robot_position, dock_position, list(beacons.keys())

def launch_setup(context, *args, **kwargs):
    # Directories
    pkg_create3_control = get_package_share_directory('irobot_create_control')
    pkg_create3_description = get_package_share_directory('hermes_create_description')
    pkg_hermes_environment = get_package_share_directory('hermes_environment')

    # Paths
    control_launch_file = PathJoinSubstitution(
        [pkg_create3_control, 'launch', 'include', 'control.py'])
    description_launch_file = PathJoinSubstitution(
        [pkg_create3_description, 'launch', 'include', 'rviz2.py'])
    dock_launch_file = PathJoinSubstitution(
        [pkg_create3_description, 'launch', 'include', 'dock.py'])
    hazards_params_yaml_file = PathJoinSubstitution(
        [pkg_hermes_environment, 'config', 'hazard_vector_params.yaml'])
    ir_intensity_params_yaml_file = PathJoinSubstitution(
        [pkg_hermes_environment, 'config', 'ir_intensity_vector_params.yaml'])
    wheel_status_params_yaml_file = PathJoinSubstitution(
        [pkg_hermes_environment, 'config', 'wheel_status_params.yaml'])
    mock_params_yaml_file = PathJoinSubstitution(
        [pkg_hermes_environment, 'config', 'mock_params.yaml'])
    beacon_params_yaml_file = os.path.join(
        pkg_hermes_environment,
        'config',
        'beacon_params.yaml'
    )
    map_params_yaml_file = os.path.join(
        pkg_hermes_environment,
        'config',
        'map_params.yaml'
    )

    # Loading the yaml config files
    rf_beacon_params = load_yaml_params(beacon_params_yaml_file).get('rf_beacon_publisher_node', {}).get('ros__parameters', {})
    map_params = load_yaml_params(map_params_yaml_file)

    # Launch configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    world_configuration_json_file = os.path.join(
        pkg_hermes_environment,
        'worlds',
        LaunchConfiguration('world').perform(context)
    )

    robot_position, dock_position, beacons = generate_world(world_configuration_json_file,
                                                            LaunchConfiguration('start').perform(context),
                                                            LaunchConfiguration('end').perform(context),
                                                            map_params)

    # In case no initial beacon was provided.
    if robot_position == None:
        robot_position = (x, y, z, yaw)

    world_path = PathJoinSubstitution(
        [pkg_hermes_environment, 'worlds', map_params['world_name']]
    )
    models_path = PathJoinSubstitution(
        [pkg_hermes_environment, 'models']
    )

    # Includes
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_launch_file])
    )
    diffdrive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([control_launch_file])
    )
    spawn_dock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dock_launch_file]),
        # The robot starts docked
        launch_arguments={'x': dock_position[0], 'y': dock_position[1], 'z': dock_position[2], 'yaw': dock_position[3]}.items(),
    )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_create3',
        arguments=['-entity',
                   'create3',
                   '-topic',
                   'robot_description',
                   '-x', robot_position[0],
                   '-y', robot_position[1],
                   '-z', robot_position[2],
                   '-Y', robot_position[3]],
        output='screen',
    )

    # Publish hazards vector
    hazards_vector_node = Node(
        package='irobot_create_toolbox',
        name='hazards_vector_node',
        executable='hazards_vector_publisher_node',
        parameters=[hazards_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Publish IR intensity vector
    ir_intensity_vector_node = Node(
        package='irobot_create_toolbox',
        name='ir_intensity_vector_node',
        executable='ir_intensity_vector_publisher_node',
        parameters=[ir_intensity_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Motion Control
    motion_control_node = Node(
        package='irobot_create_toolbox',
        name='motion_control',
        executable='motion_control',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Publish wheel status
    wheel_status_node = Node(
        package='irobot_create_toolbox',
        name='wheel_status_publisher_node',
        executable='wheel_status_publisher_node',
        parameters=[wheel_status_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Publish wheel status
    mock_topics_node = Node(
        package='irobot_create_toolbox',
        name='mock_publisher_node',
        executable='mock_publisher_node',
        parameters=[mock_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Loading the beacons
    beacon_publishers = []
    for beacon in beacons:
        beacon_publishers.append(Node(
            package='hermes_environment',
            name=beacon,
            executable='rf_beacon_publisher_node',
            parameters=[rf_beacon_params,
                        {'use_sim_time': True}],
            output='screen',
        ))

    # Adding all the nodes
    nodes = [
        # Include robot description
        robot_description,
        diffdrive_controller,
        # Add nodes to LaunchDescription
        gzserver,
        gzclient,
        spawn_robot,
        spawn_dock,
        hazards_vector_node,
        ir_intensity_vector_node,
        motion_control_node,
        wheel_status_node,
        mock_topics_node
    ]

    for beacon_publisher in beacon_publishers:
        nodes.append(beacon_publisher)

    return nodes

def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('rviz', default_value='true',
                              choices=['true', 'false'], 
                              description='Start rviz.'),
        DeclareLaunchArgument('gui', default_value='true',
                              choices=['true', 'false'],
                              description='Set "false" to run gazebo headless.'),
        DeclareLaunchArgument('world', default_value='test.json',
                              description='The environment description'),
        DeclareLaunchArgument('start', default_value='',
                              description='The initial beacon for the robot'),
        DeclareLaunchArgument('end', default_value='',
                              description='The final beacon for the robot'),
    ]

    for pose_element in ['x', 'y', 'z', 'yaw']:
        ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                                               description=f'{pose_element} component of the robot pose.'))

    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)]) 