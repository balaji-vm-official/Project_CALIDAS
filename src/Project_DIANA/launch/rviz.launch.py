from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command
import os

def generate_launch_description():

    pkgPath= FindPackageShare(package='Project_DIANA').find('Project_DIANA')
    urdfModelPath=os.path.join(pkgPath, 'description', 'robot.urdf.xacro')

    # Define use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define robot_description using xacro Command
    robot_description = ParameterValue(
        Command(['xacro ', urdfModelPath]), 
        value_type=str
    )

    robot_state_publisher = Node(
        package ='robot_state_publisher',
        executable ='robot_state_publisher',
        output ='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([

        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node 
        
    ])