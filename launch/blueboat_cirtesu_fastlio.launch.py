import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='blueboat',
        description='Name of the robot'
    )

    xacro_file = PathJoinSubstitution([
        FindPackageShare('blueboat_stonefish'),
        "urdf",
        "blueboat.xacro"
    ])

    # IMPORTANT: ahora mismo /clock no tiene publisher -> use_sim_time False
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["xacro ", xacro_file]),
            'use_sim_time': False,   # <- cambiado (antes True)
        }],
        output='screen',
    )

    namespace_action = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('stonefish_ros2'), 'launch', 'stonefish_simulator.launch.py'
                    ])
                ),
                launch_arguments={
                    'simulation_data': PathJoinSubstitution([
                        FindPackageShare('blueboat_stonefish'), 'data'
                    ]),
                    'scenario_desc': PathJoinSubstitution([
                        FindPackageShare('blueboat_stonefish'), 'scenarios', 'blueboat_cirtesu_full_tank.scn'
                    ]),
                    'simulation_rate': '100.0',
                    'window_res_x': '1200',
                    'window_res_y': '800',
                    'rendering_quality': 'high',
                }.items()
            ),
        ]
    )
    
    world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_cirtesu_base_link',
        arguments=['0', '0', '0',  '0', '0', '3.1416',  'world_ned', 'map'],
        output='screen',
    )    
    
    map_to_cirtesu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_cirtesu_base_link',
        arguments=['0', '0', '0',  '3.1416', '0', '3.1416',  'map', 'cirtesu_base_link'],
        output='screen',
    )
    
    fastlio_cfg = PathJoinSubstitution([FindPackageShare('fast_lio'), 'config', 'mid360.yaml'])
    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laser_mapping',
        output='screen',
        parameters=[fastlio_cfg, {'use_sim_time': False}],
    )
    
    cirtesu_mesh = Node(
        package='blueboat_stonefish',
        executable='cirtesu_mesh_marker.py',
        name='cirtesu_mesh_marker',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    
    ekf_yaml = PathJoinSubstitution([FindPackageShare('blueboat_stonefish'), 'config', 'ekf_fastlio_gt.yaml'])
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_fastlio_gt',
        output='screen',
        parameters=[ekf_yaml, {'use_sim_time': False}],
    )

    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('blueboat_stonefish'),
        'config',
        'blueboat_cirtesu_config.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': False}],  
        arguments=['-d', rviz_cfg],            
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        robot_state_publisher_node,
        namespace_action,
        cirtesu_mesh,
        world_to_map,
        map_to_cirtesu,
        fastlio_node,
        ekf_node,

        rviz_node,
    ])