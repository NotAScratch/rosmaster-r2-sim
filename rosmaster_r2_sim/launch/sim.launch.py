"""Start Gazebo Fortress, spawn the Rosmaster R2 and bridge its topics to ROS 2.

    ros2 launch rosmaster_r2_sim sim.launch.py                  # with the Gazebo window
    ros2 launch rosmaster_r2_sim sim.launch.py headless:=true    # server only (Jetson)
    ros2 launch rosmaster_r2_sim sim.launch.py test_actuators:=true   # Phase 1 test rig
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    pkg = get_package_share_directory('rosmaster_r2_sim')
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    test_actuators = LaunchConfiguration('test_actuators').perform(context)
    world = LaunchConfiguration('world').perform(context)

    gz_args = f"-r {'-s ' if headless else ''}{world}"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'r2.urdf.xacro'),
                 ' test_actuators:=', test_actuators]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'r2',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.005', '-Y', LaunchConfiguration('yaw')],
        output='screen')

    bridges = [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'),
                     'use_sim_time': True}])]
    if test_actuators == 'true':
        bridges.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_test',
            parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge_test.yaml'),
                         'use_sim_time': True}]))

    return [gazebo, robot_state_publisher, spawn, *bridges]


def generate_launch_description():
    pkg = get_package_share_directory('rosmaster_r2_sim')
    # Gazebo resolves package://yahboomcar_description/... by searching this path
    meshes_root = os.path.dirname(get_package_share_directory('yahboomcar_description'))

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                              description='Run the Gazebo server only, without its window'),
        DeclareLaunchArgument('test_actuators', default_value='false', choices=['true', 'false'],
                              description='Add the Phase 1 joint test rig'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'empty.sdf')),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', meshes_root),
        OpaqueFunction(function=launch_setup),
    ])
