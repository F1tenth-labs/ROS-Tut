import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Locate the parameter file within this package's 'config' directory
    params_file = os.path.join(
        get_package_share_directory('tut_pkg'),
        'config',
        'tut_params.yaml'
    )

    sensor_node = Node(
        package='tut_pkg',
        executable='sensor_node',
        name='sensor_node',  # Must match the name in the YAML file
        parameters=[params_file]
    )

    nav_node = Node(
        package='tut_pkg',
        executable='nav_node',
        name='nav_node',  # Must match the name in the YAML file
        parameters=[params_file]
    )

    motor_node = Node(
        package='tut_pkg',
        executable='motor_node',
        name='motor_node',  # Must match the name in the YAML file
        parameters=[params_file]
    )

    return LaunchDescription([
        sensor_node,
        nav_node,
        motor_node
    ])
