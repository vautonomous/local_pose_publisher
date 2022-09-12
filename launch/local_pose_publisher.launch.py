import os
import yaml

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lanelet2_map_loader_param_path = os.path.join(
        get_package_share_directory('map_loader'),
        "config",
        "lanelet2_map_loader.param.yaml",
    )
    with open(lanelet2_map_loader_param_path, "r") as f:
        lanelet2_map_loader_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    local_pose_publisher_param_path = os.path.join(
        get_package_share_directory('local_pose_publisher'),
        "config",
        "local_pose_publisher.param.yaml",
    )
    with open(local_pose_publisher_param_path, "r") as f:
        local_pose_publisher_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    return LaunchDescription([
        Node(
            package='local_pose_publisher',
            executable='local_pose_publisher_exe',
            name='local_pose_publisher_node',
            parameters=[
                local_pose_publisher_param,
                lanelet2_map_loader_param,
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
