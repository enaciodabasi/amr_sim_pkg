
import os
from pydoc import describe

import pkg_resources
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # File Paths

    pkg_share = FindPackageShare(package='amr_sim_pkg').find('amr_sim_pkg')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/gazebo.rviz')
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/amr.urdf')
    ekf_config_file_path = os.path.join(pkg_share, 'config/ekf.yaml')
    sdf_model_path = 'models/amr/model.sdf'
    

    default_world_path = os.path.join(pkg_share, 'worlds/warehouse_template.world')
    gazebo_models_path = os.path.join(pkg_share, 'models')

    #os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    sdf_model_path = os.path.join(pkg_share, sdf_model_path)

    # Launch Configs

    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    sdf_model = LaunchConfiguration('sdf_model')
    publish_joints = LaunchConfiguration('publish_joints')
    use_robot_state_publisher = LaunchConfiguration('use_robot_state_publisher')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    ld = LaunchDescription()

    # Launch Arguments

    declare_rviz_config_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Path to the RVIZ config file'
    )

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Path to the URDF model'
    )

    declare_sdf_model_path_cmd = DeclareLaunchArgument(
        name='sdf_model',
        default_value=sdf_model_path,
        description='Path to the SDF model'
    )

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Path to the .world file'
    )

    declare_use_robot_state_publisher_cmd = DeclareLaunchArgument(
        name='use_robot_state_publisher',
        default_value='True',
        description='Flag to start the Robot State Publisher'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='False',
        description='Flag to start RVIZ2'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Flag to use Gazebo time'
    )

    declare_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='publish_joints',
        default_value='True',
        description='Launch the Joint State Publisher'
    )    

    start_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
        #condition=IfCondition(publish_joints)
    )

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model])
            }]
    )

    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', default_world_path],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'amr', '-topic', '/robot_description', '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    start_robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file_path, {'use_sim_time': use_sim_time}]
    )

    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_robot_state_publisher_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_joint_state_publisher_cmd)
    ld.add_action(declare_sdf_model_path_cmd)

    ld.add_action(start_joint_state_publisher)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_rviz)
    ld.add_action(start_gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(start_robot_localization)

    return ld
