from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gtu_map_params = {
        "origin_latitude": 40.81187906,
        "origin_longitude": 29.35810110,
        "origin_altitude": 48.35
    }
    return LaunchDescription([
        Node(
            package='latlong2closestlanelet',
            executable='latlong2closestlanelet',
            name='latlong2closestlanelet',
            parameters=[gtu_map_params],
            remappings=[
                ("~/output/goal_pose_on_lanelet", "/planning/mission_planning/goal"),
            ],
            output='screen'
        )
    ])
