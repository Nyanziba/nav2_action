from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command([
                FindExecutable(name='xacro'),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('cub2_description'),
                    'urdf',
                    'cub2.urdf'
                ])
            ])}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('cub2_description'),
                'rviz',
                'display.rviz'
            ])]
        )
    ])
