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
    """Kills any orphaned Gazebo, RTAB-Map, or ROS 2 processes before starting."""
    os.system(
        "killall -9 gz sim gz-sim-server parameter_bridge static_transform_publisher "
        "rviz2 robot_state_publisher image_bridge rtabmap rgbd_odometry rtabmap_viz "
        "async_slam_toolbox_node autonomous_driver.py autonomous_racer.py "
        "auto_explorer.py ruby 2>/dev/null || true"
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
    return 'default'


def generate_launch_description():
    pkg_name = 'Project_CALIDAS'
    pkg_share = get_package_share_directory(pkg_name)

    enable_explorer = LaunchConfiguration('enable_explorer', default='true')
    declare_enable_explorer = DeclareLaunchArgument(
        'enable_explorer',
        default_value='true',
        description='Run auto_explorer.py (Single-Camera 360 Stop-and-Scan Explorer)',
    )

    # 1. Resolve path and world name for city.sdf
    world_file_path = os.path.join(pkg_share, 'worlds', 'city.sdf')
    world_name = get_world_name(world_file_path)

    # 2. Process full_auto.urdf.xacro
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

    # 4. Launch Gazebo Harmonic Server (-s -r) with city.sdf
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': f'-s -r {world_file_path}',
            'on_exit_shutdown': 'True',
        }.items(),
    )

    # 5. Spawn differential-drive robot into city.sdf
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-world', world_name,
            '-z', '0.20',
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

    # 8. Static TF aliases so Gazebo's internal scoped sensor frames match camera_link_optical
    camera_tf_bridge_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'camera_link_optical',
            '--child-frame-id', 'robot/camera_link/camera',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    camera_tf_bridge_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'camera_link_optical',
            '--child-frame-id', 'auto/camera_link/camera',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 9. ROS-Gazebo Parameter Bridge
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    # 10. Autonomous Single-Camera 360 Stop-and-Scan Explorer Node
    auto_explorer_node = Node(
        package=pkg_name,
        executable='auto_explorer.py',
        name='auto_city_explorer',
        condition=IfCondition(enable_explorer),
        parameters=[{
            'use_sim_time': True,
            'step_distance': 1.80,
            'cruise_speed': 0.45,
            'turn_speed': 0.80,
            'camera_scan_speed': 1.00,
            'stop_distance': 0.85,
        }],
        output='screen',
    )

    # 11. RTAB-Map 3D RGB-D SLAM configured for a Rotating Camera on a Stationary Base:
    #     --RGBD/LinearUpdate 0 and --RGBD/AngularUpdate 0 force RTAB-Map to map while
    #     the camera rotates 360 degrees even when base_link is completely stopped!
    rtabmap_pkg_path = get_package_share_directory('rtabmap_launch')
    rtabmap_launch_path = os.path.join(rtabmap_pkg_path, 'launch', 'rtabmap.launch.py')
    rtabmap_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'rtabmap_args': (
                '--delete_db_on_start '
                '--Reg/Force3DoF true '
                '--Grid/Sensor true '
                '--Grid/RangeMax 5.0 '
                '--RGBD/LinearUpdate 0.0 '
                '--RGBD/AngularUpdate 0.0 '
                '--Rtabmap/DetectionRate 2.0 '
                '--Mem/DepthCompressionFormat .png'
            ),
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth_image',
            'camera_info_topic': '/camera/camera_info',
            'frame_id': 'base_link',
            'odom_topic': '/odom',
            'visual_odometry': 'false',
            'approx_sync': 'true',
            'approx_rgbd_sync': 'true',
            'map_always_update': 'true',
            'qos': '2',
            'rviz': 'false',
            'rtabmap_viz': 'false',
        }.items(),
    )

    # 12. RViz2
    rviz_config_path = os.path.join(pkg_share, 'config', 'city.rviz')
    if not os.path.exists(rviz_config_path):
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
        declare_enable_explorer,
        OpaqueFunction(function=kill_stale_processes),
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        start_gui_after_spawn,
        shutdown_when_gui_closes,
        camera_tf_bridge_1,
        camera_tf_bridge_2,
        bridge,
        auto_explorer_node,
        rtabmap_slam,
        rviz_node,
    ])