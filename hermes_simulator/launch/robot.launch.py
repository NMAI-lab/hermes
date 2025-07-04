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
    pkg_hermes_agent = os.path.abspath(os.path.join(get_package_share_directory('hermes_agent'), '..', '..', '..', '..', 'src', 'hermes', 'hermes_agent'))

    # Environment config files.
    lidar_params_yaml_file = os.path.join(
        pkg_hermes_environment,
        'config',
        'lidar_params.yaml'
    )
    map_params_yaml_file = os.path.join(
        pkg_hermes_environment,
        'config',
        'map_params.yaml'
    )
    map_file = os.path.join(
        pkg_hermes_environment,
        'worlds',
        'map.json'
    )
    beacons_list_yaml_file = os.path.join(
        pkg_hermes_environment,
        'worlds',
        'beacons_list.yaml'
    )

    # Simulator config files.
    lidar_sensor_params_yaml_file = os.path.join(
        pkg_hermes_simulator,
        'config',
        'lidar_sensor_params.yaml'
    )
    beacon_sensor_params_yaml_file = os.path.join(
        pkg_hermes_simulator,
        'config',
        'beacon_sensor_params.yaml'
    )
    dock_sensor_params_yaml_file = os.path.join(
        pkg_hermes_simulator,
        'config',
        'dock_sensor_params.yaml'
    )
    bumper_sensor_params_yaml_file = os.path.join(
        pkg_hermes_simulator,
        'config',
        'bumper_sensor_params.yaml'
    )
    navigator_params_yaml_file = os.path.join(
        pkg_hermes_simulator,
        'config',
        'navigator_params.yaml'
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

    # Hermes agent config files
    agent_definitions_folder = os.path.join(
        pkg_hermes_agent,
        'agents'
    )
    hermes_agent_config_folder = os.path.join(
        pkg_hermes_agent,
        'config'
    )
    
    # Setting the environment variables
    agent_environment = os.environ.copy()
    agent_environment['agent_definitions'] = agent_definitions_folder
    agent_environment['config'] = hermes_agent_config_folder

    # Adding all the nodes
    nodes = [
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB0',
                         'serial_baudrate': 115200,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True}],
            output='screen'
        ),
        Node(package='hermes_simulator',
             namespace='perceptions',
             executable='lidar_sensor',
             name='lidar_sensor',
             output='log',
             parameters=[
                {'sensor_params': lidar_sensor_params_yaml_file},
                {'lidar_params': lidar_params_yaml_file},
                {'map_params': map_params_yaml_file}
            ]
        ),
        Node(package='hermes_simulator',
             namespace='perceptions',
             executable='beacon_sensor',
             name='beacon_sensor',
             output='log',
             parameters=[
                {'sensor_params': beacon_sensor_params_yaml_file},
                {'mode': 'bluetooth'},
                {'beacons_list': beacons_list_yaml_file}
             ]
        ),
        Node(package='hermes_simulator',
             namespace='perceptions',
             executable='dock_sensor',
             name='dock_sensor',
             output='log',
             parameters=[
                {'sensor_params': dock_sensor_params_yaml_file},
                {'mode': 'robot'}
             ]
        ),
        Node(package='hermes_simulator',
             namespace='perceptions',
             executable='bumper_sensor',
             name='bumper_sensor',
             output='log',
             parameters=[
                {'sensor_params': bumper_sensor_params_yaml_file}
             ]
        ),
        Node(package='hermes_simulator',
             namespace='navigation',
             executable='navigator',
             name='navigator',
             output='log',
             parameters=[
                {'navigator_params': navigator_params_yaml_file},
                {'map_file': map_file},
                {'destination': LaunchConfiguration('end')}
             ]
        ),
        Node(package='hermes_simulator',
             namespace='control',
             executable='action_translator',
             name='action_translator',
             output='log',
             parameters=[
                {'action_translator_params': action_translator_params_yaml_file},
                {'mode': 'robot'}
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
        ExecuteProcess(
            cmd=['ros2', 'run', 'hermes_agent', 'hermes_agent'],
            name='hermes_agent',
            output='screen',
            env=agent_environment
        )
    ]

    return nodes

def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('map', default_value='map.json',
                              description='The environment description'),
        DeclareLaunchArgument('end', default_value='',
                              description='The final beacon for the robot'),
    ]

    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)]) 