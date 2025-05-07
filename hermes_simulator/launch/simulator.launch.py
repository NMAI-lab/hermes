from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_hermes_environment = get_package_share_directory('hermes_environment')

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
        robot_environment
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