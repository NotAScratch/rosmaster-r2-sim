"""Start Gazebo Fortress, spawn the Rosmaster R2 and bridge its topics to ROS 2.

    ros2 launch rosmaster_r2_sim sim.launch.py                  # with the Gazebo window
    ros2 launch rosmaster_r2_sim sim.launch.py headless:=true    # server only (Jetson)
    ros2 launch rosmaster_r2_sim sim.launch.py publish_odom_tf:=true  # no EKF: odom TF from wheels
    ros2 launch rosmaster_r2_sim sim.launch.py world:=test_room      # walls and obstacles
    ros2 launch rosmaster_r2_sim sim.launch.py camera:=false          # no depth camera (faster)
    ros2 launch rosmaster_r2_sim sim.launch.py test_actuators:=true   # Phase 1 test rig

The robot is spawned from a converted SDF file, not from the robot_description topic:
Fortress's create-from-topic path drops <visibility_flags> from the model's visuals
(the lidar then sees its own housing and the camera), while create-from-file keeps them.
"""
import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    pkg = get_package_share_directory('rosmaster_r2_sim')
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    test_actuators = LaunchConfiguration('test_actuators').perform(context)
    camera = LaunchConfiguration('camera').perform(context)
    world = LaunchConfiguration('world').perform(context)
    if os.sep not in world:
        # A bare name means one of this package's worlds
        world = os.path.join(pkg, 'worlds', world if world.endswith('.sdf') else f'{world}.sdf')

    gz_args = f"-r {'-s ' if headless else ''}{world}"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    # xacro -> URDF (for robot_state_publisher) -> SDF (for Gazebo). Spawning from the
    # URDF topic instead would lose the visuals' visibility_flags (see the module docstring).
    urdf = subprocess.run(
        ['xacro', os.path.join(pkg, 'urdf', 'r2.urdf.xacro'),
         f'test_actuators:={test_actuators}', f'camera:={camera}'],
        capture_output=True, text=True, check=True).stdout
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf, 'use_sim_time': True}])

    urdf_file = tempfile.NamedTemporaryFile(prefix='r2_', suffix='.urdf', delete=False)
    urdf_file.write(urdf.encode())
    urdf_file.close()
    sdf_file = tempfile.NamedTemporaryFile(prefix='r2_', suffix='.sdf', delete=False)
    sdf = subprocess.run(['ign', 'sdf', '-p', urdf_file.name],
                         capture_output=True, text=True, check=True).stdout
    sdf_file.write(sdf.encode())
    sdf_file.close()

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', sdf_file.name, '-name', 'r2',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.005', '-Y', LaunchConfiguration('yaw')],
        output='screen')

    bridge_configs = ['bridge', 'bridge_sensors']
    if camera == 'true':
        bridge_configs.append('bridge_camera')
    if test_actuators == 'true':
        bridge_configs.append('bridge_test')
    else:
        bridge_configs.append('bridge_drive')
        if LaunchConfiguration('publish_odom_tf').perform(context) == 'true':
            bridge_configs.append('bridge_odom_tf')

    bridges = [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=name,
        parameters=[{'config_file': os.path.join(pkg, 'config', f'{name}.yaml'),
                     'use_sim_time': True}]) for name in bridge_configs]

    return [gazebo, robot_state_publisher, spawn, *bridges]


def generate_launch_description():
    # Gazebo resolves package://yahboomcar_description/... by searching this path
    meshes_root = os.path.dirname(get_package_share_directory('yahboomcar_description'))

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                              description='Run the Gazebo server only, without its window'),
        DeclareLaunchArgument('test_actuators', default_value='false', choices=['true', 'false'],
                              description='Replace the Ackermann drive with the Phase 1 joint test rig'),
        DeclareLaunchArgument('publish_odom_tf', default_value='false', choices=['true', 'false'],
                              description='Publish odom -> base_footprint from wheel odometry '
                                          '(leave false when an EKF publishes it)'),
        DeclareLaunchArgument('world', default_value='empty',
                              description="A world in this package ('empty', 'test_room') or a path to an .sdf file"),
        DeclareLaunchArgument('camera', default_value='true', choices=['true', 'false'],
                              description='Simulate the Astra Plus colour and depth camera'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', meshes_root),
        OpaqueFunction(function=launch_setup),
    ])
