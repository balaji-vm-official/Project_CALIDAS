import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    EmitEvent,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import xacro


def generate_launch_description():
    pkg_name = 'Project_CALIDAS'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Path to custom race track SDF world
    world_file_path = os.path.join(pkg_share, 'worlds', 'city.sdf')

    # 2. Process depth_full_auto.urdf.xacro (Ackermann car with depth depth_camera + lidar)
    xacro_file_path = os.path.join(pkg_share, 'description', 'full_auto.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 4. Launch Gazebo Harmonic Server (-s -r) with race_track.sdf
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': f'-s -r {world_file_path}',
            'on_exit_shutdown': 'True',
        }.items(),
    )

    # 5. Spawn full_car on the start/finish line of race_track.sdf
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-z', '0.2'
        ],
        output='screen',
    )

    # 6. Start Gazebo GUI after the car entity exists
    gz_gui = ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen')

    start_gui_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=2.0, actions=[gz_gui])],
        )
    )

    # 7. Shut down launch cleanly when the Gazebo GUI window closes
    shutdown_when_gui_closes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_gui,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    # 8. ROS-Gazebo Parameter Bridge (/cmd_vel, /odom, /tf, /joint_states, /clock)
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # 9. High-throughput depth_camera Image Bridge (/depth_camera/image_raw)
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/depth_camera/image_raw'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 10. Autonomous depth_camera-Only Driver Node
    autonomous_driver_node = Node(
        package=pkg_name,
        executable='autonomous_driver.py',
        name='autonomous_track_driver',
        parameters=[
            {
                'use_sim_time': True,
                'wheel_base': 0.76,
                'max_steer_angle': 0.58,
                'max_speed': 1.65,
                'min_speed': 0.70,
                'kp_near': 0.52,
                'kp_far': 0.48,
                'kd_steer': 0.14,
                'show_cv_window': True,
            }
        ],
        output='screen',
    )

    # 11. Launch RViz2
    rviz_config_path = os.path.join(pkg_share, 'config', 'city.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        start_gui_after_spawn,
        shutdown_when_gui_closes,
        bridge,
        ros_gz_image_bridge,
        # autonomous_driver_node,
        rviz_node,
    ])