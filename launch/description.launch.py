import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare(package='amr_sim_pkg').find('amr_sim_pkg')
    rviz_config_path = os.path.join(pkg_share, 'rviz/visualize_urdf.rviz')
    urdf_model_path = os.path.join(pkg_share, 'urdf/amr.urdf')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_model_path,
            description='URDF path'
        )
        
    declare_joint_state_publ_cmd = DeclareLaunchArgument(
        name='publish_joints', 
        default_value='true',
        description='Launch joint_states_publisher'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        name='rviz', 
        default_value='true',
        description='Run rviz'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='true',
        description='Use simulation time'
    )

    declare_joint_state_pub_gui_cmd = DeclareLaunchArgument(
        name="gui",
        default_value='true',
        description='Run Joint State Publisher GUI'
    )

    start_joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration("publish_joints")),
        output='screen',
        parameters=[
             {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    start_joint_state_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui')
    

    start_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
            }
        ]
    )

    start_rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    

    ld = LaunchDescription()

    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_joint_state_publ_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_joint_state_pub_gui_cmd)

    ld.add_action(start_joint_state_pub)
    ld.add_action(start_joint_state_pub_gui)
    ld.add_action(start_robot_state_pub)
    ld.add_action(start_rviz)

    return ld