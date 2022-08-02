
import os
from pathlib import Path
from click import launch

from setuptools import Command
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = FindPackageShare('amr_sim_pkg').find('amr_sim_pkg')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2.rviz')
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/amr.urdf')
    default_nav2_config_path = os.path.join(pkg_share, 'config/nav_alt.yaml')
    default_world_path = os.path.join(pkg_share, 'worlds/walls.world')
    default_ekf_config_file_path = os.path.join(pkg_share, 'config/ekf.yaml')
    # add map file path

    nav2_launch_path = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])

    # Configs

    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    publish_joints = LaunchConfiguration('publish_joints')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_publisher')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')
    slam = LaunchConfiguration('slam')
    autostart = LaunchConfiguration('autostart')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    # LaunchDescription Variable

    ld = LaunchDescription()

    # Launch Args

    declare_rviz_config_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Path to the RVIZ config file'
    )


    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart',
        default_value='True',
        description='Automatically startup the NAV2 Stack'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        name='nav2_params_file',
        default_value=default_nav2_config_path,
        description='Path to NAV2 Parameter file'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Flag to start RVIZ2'
    )


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Whether to use Gazebo time'
    )

    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Nodes and Processes

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments= { 'map': '',
                            'use_sim_time': use_sim_time,
                            'params_file': nav2_params_file,
                            'autostart': autostart}.items()
    )

    ld.add_action(start_rviz)
    ld.add_action(start_nav2)

    return ld





