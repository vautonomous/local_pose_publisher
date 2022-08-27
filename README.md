### Description

This package converts GNSS-INS data (latitude,longitude,altitude) to a point on cartesian local
coordinate system and calculates & publishes the closest pose to the point in the center line of
lanelet2 map.

### How it works

The first given location information in config file is set as the map origin. Using this origin
subscribed lat-lon-alt coordinates are transformed to local coordinate system 

### Usage

`ros2 launch latlong2closestlanelet latlong2closestlanelet.launch.py`

### Input Topics

| Name                           | Type                        | Description                           |
| -------------------------------| ----------------------------| --------------------------------------|
| `~/input/goal_gnss_coordinate` | sensor_msgs::msg::NavSatFix | goal point coordinates as lat-lon-alt |

### Output Topics

| Name                            | Type                             | Description                     |
| --------------------------------| ---------------------------------| --------------------------------|
| `~/output/goal_pose_on_lanelet` | geometry_msgs::msg::PoseStamped  | closest pose in the center line |

### External Dependencies

* GeographicLib
