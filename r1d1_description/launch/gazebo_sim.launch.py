from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    # Locate the 'r1d1_description' package share directory
    r1d1_description_share = get_package_share_directory('r1d1_description')
    # Path to the custom world file
    world_file = os.path.join(r1d1_description_share, 'worlds', 'new_world1.world')
    map_yaml_file = os.path.join(r1d1_description_share, 'maps', 'map_1.yaml')
    # Verify that the world file exists
    if not os.path.exists(world_file):
        raise FileNotFoundError(f"World file not found: {world_file}")
    if not os.path.exists(map_yaml_file):
        raise FileNotFoundError(f"map file not found: {map_yaml_file}")

    # Include the Gazebo launch file with the custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # Include the Nav2 bringup launch file
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('nav2_bringup'),
    #             'launch',
    #             'bringup_launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'map': map_yaml_file
    #         'use_sim_time': True
    #     }.items()
    # )


    # Process the robot's Xacro file to generate URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),' ',
            PathJoinSubstitution([
                FindPackageShare('r1d1_description'), 'urdf', 'r1d1.xacro'])
        ]
    )
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Node to publish the robot's state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Node to spawn the robot entity in Gazebo
    robot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'r1d1'],
        output='screen'
    )

    # Load the joint state broadcaster controller
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Load the joint trajectory controller
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    # Path to the RViz configuration file
    rviz_config_file = os.path.join(r1d1_description_share, 'config', 'display.rviz')

    # Verify that the RViz config file exists
    if not os.path.exists(rviz_config_file):
        raise FileNotFoundError(f"RViz config file not found: {rviz_config_file}")

    # Node to launch RViz2 with the specified configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Optional: Delay robot spawn to ensure Gazebo is ready
    delayed_robot_spawn = TimerAction(
        period=2.0,
        actions=[robot_spawn_node]
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link1_broadcaster',
        arguments=['0', '0.025', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        delayed_robot_spawn,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        static_transform_publisher_node,
        # nav2_launch
        rviz_node
    ])
