### Description

This package converts GNSS data (latitude,longitude,altitude) to a point (raw local point) on
cartesian local coordinate system. Then gets the closest lanelet around the raw local point.
Calculates the closest center line point index to the raw local point. Generates and publishes the
center line point as `geometry_msgs::msg::PoseStamped` with the lane direction/orientation.

### Installation

```bash
cd <your_workspace>/src
git clone git@github.com:leo-drive/local_pose_publisher.git
source /opt/ros/galactic/setup.bash
cd ..
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

### Usage

* **! Update parameters in `<your_workspace>/src/local_pose_publisher/launch/local_pose_publisher.launch.py`**

```bash
cd <your_workspace>
source install/setup.bash
ros2 launch local_pose_publisher local_pose_publisher.launch.py
```

* Launch autoware.universe

### Parameters

| Name                      | Type      | Description                                                       |
| --------------------------| ----------| ------------------------------------------------------------------|
| `origin_latitude`         | double    | latitude of origin point                                          |
| `origin_longitude`        | double    | longitude of origin point                                         |
| `origin_altitude`         | double    | altitude of origin point                                          |
| `lanelet2_map_path`       | string    | path to lanelet2 map                                              |
| `center_line_resolution`  | double    | resolution of recreated fine center line                          |
| `distance_threshold`      | double    | raw goal/checkpoint point distance threshold to closest lanelet   |
| `debug_mode`              | bool      | enables debugging options and printing debug info                 |
| `nearest_lanelet_count`   | int       | closest lanelet count around raw goal/checkpoint point            |

### Input Topics

| Name                                  | Type                              | Description                                                       |
| --------------------------------------| ----------------------------------| ------------------------------------------------------------------|
| `~/input/goal_gnss_coordinate`        | sensor_msgs::msg::NavSatFix       | goal point coordinates as lat-lon-alt                             |
| `~/input/checkpoint_gnss_coordinate`  | sensor_msgs::msg::NavSatFix       | checkpoint point coordinates as lat-lon-alt                       |
| `~/input/debug/pose`                  | geometry_msgs::msg::PoseStamped   | (debugging option) stamped pose msg input besides NavSatFix msg   |

### Output Topics

| Name                                  | Type                              | Description                               |
| --------------------------------------| ----------------------------------| ------------------------------------------|
| `~/output/goal_pose_on_lanelet`       | geometry_msgs::msg::PoseStamped   | goal pose on the lanelet centerline       |
| `~/output/checkpoint_pose_on_lanelet` | geometry_msgs::msg::PoseStamped   | checkpoint pose on the lanelet centerline |
| `~/output/debug/raw_local_point`      | geometry_msgs::msg::PointStamped  | raw point on local coordinate frame       |

### Debugging

* For the sake of easier debugging of the package, goal point inputs can be given
  as `geometry_msgs::msg::PoseStamped` msg. You can specify `2D Goal Pose` topic name in rviz
  as `~/input/debug/pose`. That way the package simulates the algorithm as if a goal
  input (`sensor_msgs::msg::NavSatFix`) is given.

* In addition to that a `geometry_msgs::msg::PointStamped` is published to be able to see the raw
  point on local coordinate system.

* Observe that extra information is printed on the terminal for debugging.

### Publish Dummy NavSatFix Msg

To be able to publish required goal/checkpoint `sensor_msgs/msg/NavSatFix` msgs without running HMI,
add following functions (`pub_goal_lat_lon` & `pub_cp_lat_lon`) to your bash_aliases file:

###### Publish Goal Coordinates

```bash
pub_goal_lat_lon(){
IFS=","
read -a c_a <<< $1
lat=${c_a[0]}
lon=${c_a[1]}

if [ -z "$lon" ]
then
	read -a l_a <<< $2
	if [[ ${#l_a[@]} == 1 ]] && [ -z "${l_a[0]}" ]
	then
		lon=$3
	else
		lon=${l_a[-1]}
	fi
fi

echo "Publishing for $lat,$lon"

ros2 topic pub --once /hmi/goal_gnss_coordinate sensor_msgs/msg/NavSatFix "{ \
header: { \
frame_id: 'map'}, \
latitude: $lat, \
longitude: $lon, \
altitude: 48.35, \
}" >/dev/null
}
```

###### Publish Checkpoint Coordinates

```bash
pub_cp_lat_lon(){
IFS=","
read -a c_a <<< $1
lat=${c_a[0]}
lon=${c_a[1]}

if [ -z "$lon" ]
then
	read -a l_a <<< $2
	if [[ ${#l_a[@]} == 1 ]] && [ -z "${l_a[0]}" ]
	then
		lon=$3
	else
		lon=${l_a[-1]}
	fi
fi

echo "Publishing for $lat,$lon"

ros2 topic pub --once /hmi/checkpoint_gnss_coordinate sensor_msgs/msg/NavSatFix "{ \
header: { \
frame_id: 'map'}, \
latitude: $lat, \
longitude: $lon, \
altitude: 48.35, \
}" >/dev/null
}
```

Then run the following commands on your terminal with desired lat lon coordinate arguments:

`pub_goal_lat_lon 40.814364, 29.36180
`

`pub_cp_lat_lon 40.811057, 29.359412
`

### External Dependencies

* geographiclib
* libboost-dev
* eigen