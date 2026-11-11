import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import xacro

def generate_launch_description():
    pkg_name = 'Project_DIANA'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Process xacro
    xacro_file_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 2. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 3. Gazebo Sim with -r to run immediately (unpaused)
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # 4. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'my_robot',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Launch the ROS-Gazebo bridge for normal topics
    bridge_params = os.path.join(get_package_share_directory(pkg_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        ros_gz_bridge
    ])