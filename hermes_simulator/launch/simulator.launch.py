from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

import os

def launch_setup(context, *args, **kwargs):
    pkg_hermes_environment = get_package_share_directory('hermes_environment')
    pkg_hermes_simulator = get_package_share_directory('hermes_simulator')

    # Environment config files.
    lidar_params_yaml_file = os.path.join(
        pkg_hermes_environment,
        'config',
        'lidar_params.yaml'
    )
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
    lidar_sensor_params_yaml_file = os.path.join(
        pkg_hermes_simulator,
        'config',
        'lidar_sensor_params.yaml'
    )
    action_translator_params_yaml_file = os.path.join(
        pkg_hermes_simulator,
        'config',
        'action_translator_params.yaml'
    )
    belief_generator_params_yaml_file = os.path.join(
        pkg_hermes_simulator,
        'config',
        'belief_generator_params.yaml'
    )

    environment_launch_file = PathJoinSubstitution(
        [pkg_hermes_environment, 'launch', 'environment.launch.py'])

    # Includes
    robot_environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([environment_launch_file]),
        launch_arguments={'rviz': LaunchConfiguration('rviz'),
                          'gui': LaunchConfiguration('gui'),
                          'map': LaunchConfiguration('map'),
                          'start': LaunchConfiguration('start'),
                          'end': LaunchConfiguration('end'),
                          'x': LaunchConfiguration('x'),
                          'y': LaunchConfiguration('y'),
                          'z': LaunchConfiguration('z'),
                          'yaw': LaunchConfiguration('yaw')}.items(),
    )
    
    # Adding all the nodes
    nodes = [
        robot_environment,
        Node(package='hermes_simulator',
             namespace='perceptions',
             executable='lidar_sensor',
             name='lidar_sensor',
             output='log',
             parameters=[
                {'sensor_params': lidar_sensor_params_yaml_file},
                {'lidar_params': lidar_params_yaml_file}
            ]
        ),
        Node(package='hermes_simulator',
             namespace='perceptions',
             executable='beacon_sensor',
             name='beacon_sensor',
             output='log',
             parameters=[beacon_params_yaml_file]
        ),
        Node(package='hermes_simulator',
             namespace='control',
             executable='action_translator',
             name='action_translator',
             output='log',
             parameters=[
                {'action_translator_params': action_translator_params_yaml_file}
            ]
        ),
        Node(package='hermes_simulator',
             namespace='control',
             executable='belief_generator',
             name='belief_generator',
             output='log',
             parameters=[
                {'belief_generator_params': belief_generator_params_yaml_file}
            ]
        ),
    ]

    return nodes

def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('rviz', default_value='true',
                              choices=['true', 'false'], 
                              description='Start rviz.'),
        DeclareLaunchArgument('gui', default_value='true',
                              choices=['true', 'false'],
                              description='Set "false" to run gazebo headless.'),
        DeclareLaunchArgument('map', default_value='map.json',
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