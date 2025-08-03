import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Simulation-only launch for development and testing."""
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    
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
            'gui',
            default_value='true',
            description='Start Gazebo GUI if true'
        )
    )
    
    # Robot description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('graspnet_description'),
                'launch',
                'display.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': 'false'
        }.items()
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
            'use_sim_time': use_sim_time,
            'headless': LaunchConfiguration('gui', default='false'),
        }.items()
    )
    
    # RViz for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('graspnet_bringup'),
        'config',
        'sim_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription(
        declared_arguments + [
            robot_description_launch,
            gazebo_launch,
            rviz_node,
        ]
    )
