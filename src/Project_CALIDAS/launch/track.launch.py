import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    EmitEvent,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import xacro


def kill_stale_processes(context, *args, **kwargs):
    """Kills any orphaned Gazebo or ROS 2 processes from previous runs before starting."""
    os.system(
        "killall -9 gz sim gz-sim-server parameter_bridge static_transform_publisher "
        "rviz2 robot_state_publisher image_bridge autonomous_driver.py "
        "autonomous_racer.py auto_explorer.py ruby 2>/dev/null || true"
    )
    return []


def get_world_name(sdf_path):
    """Parses the <world name='...'> attribute from the SDF file so spawn_entity never times out."""
    try:
        with open(sdf_path, 'r') as f:
            match = re.search(r'<world\s+name=["\']([^"\']+)["\']', f.read())
            if match:
                return match.group(1)
    except Exception:
        pass
    return 'flat_circuit'


def generate_launch_description():
    pkg_name = 'Project_CALIDAS'  # Change to 'Project_CALIDAS' if you rename package.xml
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Resolve path and world name for track.sdf
    world_file_path = os.path.join(pkg_share, 'worlds', 'track.sdf')
    world_name = get_world_name(world_file_path)

    # 2. Compile full_car.urdf.xacro (Ackermann car + camera + lidar) into XML string
    xacro_file_path = os.path.join(pkg_share, 'description', 'full_car.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. Robot State Publisher (broadcasts URDF link transforms over /tf and /tf_static)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 4. Start Gazebo Harmonic Server (-s -r) with track.sdf
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': f'-s -r {world_file_path}',
            'on_exit_shutdown': 'True',
        }.items(),
    )

    # 5. Spawn Ackermann car on the start/finish straight of track.sdf
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-world', world_name,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.20',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # 6. Start Gazebo GUI 2.0s after the robot entity spawns
    gz_gui = ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen')

    start_gui_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=2.0, actions=[gz_gui])],
        )
    )

    # 7. Shut down the entire launch stack cleanly when the Gazebo GUI closes
    shutdown_when_gui_closes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_gui,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    # 8. ROS-Gazebo Parameter Bridge (/clock, /cmd_vel, /odom, /tf, /joint_states)
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    # 9. High-throughput Camera Image Bridge (/camera/image_raw)
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 10. Autonomous OpenCV Camera Driver Node
    autonomous_driver_node = Node(
        package=pkg_name,
        executable='autonomous_driver.py',
        name='autonomous_track_driver',
        parameters=[{
            'use_sim_time': True,
            'wheel_base': 0.76,
            'max_steer_angle': 0.58,
            'max_speed': 1.65,
            'min_speed': 0.70,
            'show_cv_window': True,
        }],
        output='screen',
    )

    # 11. RViz2
    rviz_config_path = os.path.join(pkg_share, 'config', 'lidar_only.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1'),
        SetEnvironmentVariable(name='IGN_IP', value='127.0.0.1'),
        OpaqueFunction(function=kill_stale_processes),
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        start_gui_after_spawn,
        shutdown_when_gui_closes,
        bridge,
        ros_gz_image_bridge,
        autonomous_driver_node,
        rviz_node,
    ])