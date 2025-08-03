import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Complete GraspNet system launch file."""
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_moveit = LaunchConfiguration('use_moveit')
    world_file = LaunchConfiguration('world_file')
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz if true'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit',
            default_value='true',
            description='Start MoveIt if true'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('graspnet_gazebo'),
                'worlds',
                'graspnet_world.world'
            ]),
            description='World file to load in Gazebo'
        )
    )
    
    # Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('graspnet_gazebo'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world_file': world_file,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # MoveIt motion planning
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('graspnet_moveit'),
                'launch',
                'moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_moveit)
    )
    
    # Control nodes
    robot_controller_node = Node(
        package='graspnet_control',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    grasp_planner_node = Node(
        package='graspnet_control',
        executable='grasp_planner',
        name='grasp_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Perception nodes
    camera_manager_node = Node(
        package='graspnet_perception',
        executable='camera_manager',
        name='camera_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    object_detector_node = Node(
        package='graspnet_perception',
        executable='object_detector',
        name='object_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('graspnet_bringup'),
        'config',
        'graspnet.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create the launch description
    return LaunchDescription(
        declared_arguments + [
            gazebo_launch,
            moveit_launch,
            robot_controller_node,
            grasp_planner_node,
            camera_manager_node,
            object_detector_node,
            rviz_node,
        ]
    )
