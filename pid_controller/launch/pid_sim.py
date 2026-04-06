from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('kp', default_value='0.02'),
        DeclareLaunchArgument('ki', default_value='0.006'),
        DeclareLaunchArgument('kd', default_value='0.002'),
        Node(
            package='pid_controller',
            executable='PCN',
            name='p_controller',
            parameters=[{
            'p': LaunchConfiguration('kp'),
            'i': LaunchConfiguration('ki'),
            'd': LaunchConfiguration('kd'),
         }]
        ),
        Node(
            package='joint_sim',
            executable='joint_sim_node',
            name='sim',
            parameters=[PathJoinSubstitution([
            FindPackageShare('pid_controller'), 'launch/config', 'parameter.yaml'])
            ],
        ),
        Node(
            package='pid_controller',
            executable='RIN',
            name='reference_input',
            emulate_tty=True,
        )
    ])