from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    isuzu_map_params = {
        "latitude_map": 40.81187906,
        "longitude_map": 29.35810110,
        "altitude_map": 48.35
    }
    return LaunchDescription([
        Node(
            package='latlong2closestlanelet',
            executable='latlong2closestlanelet',
            name='latlong2closestlanelet',
            parameters=[isuzu_map_params],
            output='screen'
        )

    ])
