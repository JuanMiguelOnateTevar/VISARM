from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm',
            executable='node_arm',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='camera_processing',
            executable='camera_node',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='esp32',
            executable='node_esp32',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='frontend',
            executable='node_ui',
            output='screen',
            emulate_tty=True
        )
    ])