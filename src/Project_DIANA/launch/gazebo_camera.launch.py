import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import xacro

from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    pkg_name = 'Project_DIANA'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Define the absolute path to your custom SDF world file
    world_file_path = os.path.join(pkg_share, 'worlds', 'silverstone.sdf')

    # 2. Process your robot xacro file
    xacro_file_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 4. Include Gazebo Sim and pass your custom world path
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': f'-s -r {world_file_path}',
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # 5. Spawn your robot into the custom world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'robot',
            '-z', '0.3'
        ],
        output='screen'
    )

    # 6. GUI starts only after the robot exists, so it receives the lidar entity
    gz_gui = ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen')

    start_gui_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=2.0, actions=[gz_gui])],
        )
    )

    # 7. Closing the GUI window shuts down the whole launch
    shutdown_when_gui_closes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_gui,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    # 8. Launch the ROS-Gazebo bridge
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    # 9. Launch RViz2 with use_sim_time
    rviz_config_path = os.path.join(pkg_share, 'config', 'lidar_only.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 10. Dedicated Image Bridge for high-throughput camera frames
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        start_gui_after_spawn,
        shutdown_when_gui_closes,
        bridge,
        rviz_node,
        ros_gz_image_bridge
    ])