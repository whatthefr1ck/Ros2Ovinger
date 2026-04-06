from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid_controller',
            namespace='pid_controller1',
            executable='PCN',
            name='pid_sim',
            
        ),
        Node(
            package='joint_sim',
            namespace='joint_sim1',
            executable='joint_sim_node',
            name='joint_sim',
            
        ),
        Node(
            package='pid_controller',
            executable='RIN',
            name='reference_input_node',
        )
    ])