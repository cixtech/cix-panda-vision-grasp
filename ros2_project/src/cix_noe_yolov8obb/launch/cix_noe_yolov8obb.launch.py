from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cix_noe_yolov8obb',
            executable='cix_noe_yolov8obb_publisher.py',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
