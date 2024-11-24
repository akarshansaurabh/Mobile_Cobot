from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('r1d1_description'),
            'worlds', 'new_world1.world'),
        description='Full path to the world model file to load')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('r1d1_description'),
            'maps', 'map_1.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # Get the launch configuration variables
    world_file = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include the Gazebo launch file with the custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # Process the robot's Xacro file to generate URDF
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('r1d1_description'), 'urdf', 'r1d1.xacro'
            ]),
            ' ',
            'use_sim_time:=', use_sim_time
        ]
    )
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Node to publish the robot's state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Node to spawn the robot entity in Gazebo
    robot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'r1d1'],
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

    # Include the Nav2 bringup launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time
        }.items()
    )


    # Optional: Delay robot spawn to ensure Gazebo is ready
    delayed_robot_spawn = TimerAction(
        period=5.0,
        actions=[robot_spawn_node]
    )

    # Static transform publisher from map to odom
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    rviz_config_file = os.path.join(get_package_share_directory('r1d1_description'), 'config', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the declared launch arguments
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to launch all the nodes
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(delayed_robot_spawn)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_joint_trajectory_controller)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_node)

    return ld
