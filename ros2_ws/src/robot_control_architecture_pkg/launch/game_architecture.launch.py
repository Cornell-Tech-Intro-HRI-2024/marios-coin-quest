from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    reactive_node = Node(
        package='robot_control_architecture_pkg',
        executable='reactive_architecture',
        name='reactive_architecture_node',
        output='screen'
    )

    deliberative_node = Node(
        package='robot_control_architecture_pkg',
        executable='deliberative_architecture',
        name='deliberative_architecture_node',
        output='screen'
    )

    return LaunchDescription([
        reactive_node,
        deliberative_node
    ])