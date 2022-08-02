import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    slam_launch_path = PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
    pkg_share = FindPackageShare(package='amr_sim_pkg').find('amr_sim_pkg')
    slam_config_path = os.path.join(pkg_share, 'config/slam.yaml')
    slam_rviz_config_path = os.path.join(pkg_share, 'rviz/slam.rviz')
    slam_param_name = slam_config_path

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use Gazebo clock.'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Start RVIZ2'
    )

    start_online_aync_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time':use_sim_time,
            slam_param_name: slam_config_path
        }.items()
    )

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', slam_rviz_config_path],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)

    ld.add_action(start_online_aync_slam)
    ld.add_action(start_rviz)

    return ld