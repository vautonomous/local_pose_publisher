from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gtu_map_params = {
        "origin_latitude": 40.81187906,
        "origin_longitude": 29.35810110,
        "origin_altitude": 48.35
    }
    node_params = {
        "lanelet2_map_path": "/home/md/projects/operational-design-domains/public_road_bus/maps/gebze/lanelet2_map.osm",
        "center_line_resolution": 1.0,  # [m]
        "distance_threshold": 30.0,  # [m]
        "debug_mode": False,
    }
    advanced_node_params = {
        "nearest_lanelet_count": 10,
    }
    return LaunchDescription([
        Node(
            package='local_pose_publisher',
            executable='local_pose_publisher_exe',
            name='local_pose_publisher_node',
            parameters=[
                gtu_map_params,
                node_params,
                advanced_node_params,
            ],
            remappings=[
                ("~/input/vector_map", "/map/vector_map"),
                ("~/input/goal_gnss_coordinate", "/hmi/goal_gnss_coordinate"),
                ("~/input/checkpoint_gnss_coordinate", "/hmi/checkpoint_gnss_coordinate"),
                ("~/input/debug/pose", "/local_pose_publisher_node/debug/pose"),
                ("~/output/goal_pose_on_lanelet", "/planning/mission_planning/goal"),
                ("~/output/checkpoint_pose_on_lanelet", "/planning/mission_planning/checkpoint"),
            ],
            output='screen'
        )
    ])
