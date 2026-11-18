import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    EmitEvent,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import xacro


def kill_stale_processes(context, *args, **kwargs):
    """Cleans up any lingering Gazebo, SLAM, or Nav2 processes before starting."""
    os.system(
        "killall -9 gz sim gz-sim-server parameter_bridge static_transform_publisher "
        "rviz2 robot_state_publisher image_bridge async_slam_toolbox_node "
        "autonomous_driver.py autonomous_racer.py auto_explorer.py ruby 2>/dev/null || true"
    )
    return []


def get_world_name(sdf_path):
    try:
        with open(sdf_path, 'r') as f:
            match = re.search(r'<world\s+name=["\']([^"\']+)["\']', f.read())
            if match:
                return match.group(1)
    except Exception:
        pass
    return 'silverstone_grey_world'


def generate_launch_description():
    pkg_name = 'Project_CALIDAS'  # Change to 'Project_CALIDAS' if you rename package.xml
    pkg_share = get_package_share_directory(pkg_name)

    # Launch arguments so you can switch between Reactive Racing+SLAM vs Goal-Based Nav2
    enable_racer = LaunchConfiguration('enable_racer', default='true')
    use_nav2 = LaunchConfiguration('use_nav2', default='false')

    declare_enable_racer = DeclareLaunchArgument(
        'enable_racer',
        default_value='true',
        description='Run autonomous_racer.py (Follow-the-Gap LiDAR racer) automatically',
    )
    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='false',
        description='Launch the Nav2 navigation stack alongside SLAM Toolbox',
    )

    # 1. Resolve path and world name for silverstone.sdf
    world_file_path = os.path.join(pkg_share, 'worlds', 'silverstone.sdf')
    world_name = get_world_name(world_file_path)

    # 2. Process full_car.urdf.xacro
    xacro_file_path = os.path.join(pkg_share, 'description', 'full_car.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 4. Launch Gazebo Harmonic Server (-s -r) with silverstone.sdf
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': f'-s -r {world_file_path}',
            'on_exit_shutdown': 'True',
        }.items(),
    )

    # 5. Spawn Ackermann car into silverstone_grey_world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-world', world_name,
            '-z', '0.30',
        ],
        output='screen',
    )

    # 6. Start Gazebo GUI 2.0s after spawn
    gz_gui = ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen')

    start_gui_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=2.0, actions=[gz_gui])],
        )
    )

    # 7. Clean shutdown when Gazebo GUI closes
    shutdown_when_gui_closes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_gui,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    # 8. Static TF alias in case Gazebo scopes the laser frame as robot/laser_frame/laser
    lidar_tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'laser_frame',
            '--child-frame-id', 'robot/laser_frame/laser',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 9. ROS-Gazebo Parameter Bridge (/clock, /cmd_vel, /odom, /tf, /joint_states, /scan)
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    # 10. SLAM Toolbox (online_async_launch.py) for real-time 2D LiDAR mapping
    slam_toolbox_pkg_path = get_package_share_directory('slam_toolbox')
    slam_launch_path = os.path.join(slam_toolbox_pkg_path, 'launch', 'online_async_launch.py')
    slam_params_path = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': slam_params_path,
            'slam_params_file': slam_params_path,
        }.items(),
    )

    # 11. Optional Nav2 Bringup (enabled when launched with use_nav2:=true)
    nav2_pkg_path = get_package_share_directory('nav2_bringup')
    nav2_launch_path = os.path.join(nav2_pkg_path, 'launch', 'navigation_launch.py')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path,
        }.items(),
    )

    # 12. Autonomous LiDAR Racer Node (Follow-the-Gap + Wall-Centering)
    autonomous_racer_node = Node(
        package=pkg_name,
        executable='autonomous_racer.py',
        name='autonomous_lidar_racer',
        condition=IfCondition(enable_racer),
        parameters=[{
            'use_sim_time': True,
            'wheel_base': 0.76,
            'max_steer_angle': 0.58,
            'max_speed': 2.00,
            'min_speed': 0.75,
        }],
        output='screen',
    )

    # 13. RViz2
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
        declare_enable_racer,
        declare_use_nav2,
        OpaqueFunction(function=kill_stale_processes),
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        start_gui_after_spawn,
        shutdown_when_gui_closes,
        lidar_tf_bridge,
        bridge,
        slam_toolbox,
        nav2_navigation,
        autonomous_racer_node,
        rviz_node,
    ])