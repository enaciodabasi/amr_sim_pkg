# Author: Eren Naci Odabasi
# Date: 19/07/2022
# Description: Launch file to visualize the URDF of the AMR

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare(package='amr_sim_pkg').find('amr_sim_pkg')
    rviz_config_path = os.path.join(pkg_share, 'rviz/visualize_urdf.rviz')
    urdf_model_path = os.path.join(pkg_share, 'urdf/amr.urdf')

    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_publisher = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(name='urdf_model', default_value=urdf_model_path, description='Path to AMR URDF file.')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(name='rviz_config_file', default_value=rviz_config_path, description='Full path to RVIZ config file.')
    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher GUI.')
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(name='use_robot_state_pub', default_value='True', description='Flag to start the robot state publisher.')
    declare_use_rviz_cmd = DeclareLaunchArgument(name='use_rviz', default_value='True', description='Flag to start RVIZ.')
    declare_use_sim_time_cmd = DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Use simulation time.')

    start_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
        #condition=IfCondition(publish_joints)
    )
    robot_state_publisher_cmd = Node(condition=IfCondition(use_robot_state_publisher), package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', urdf_model])}], arguments=[urdf_model_path])
    start_rviz_cmd = Node(condition=IfCondition(use_rviz), package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', rviz_config_file])

    ld = LaunchDescription()
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(start_joint_state_publisher)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld