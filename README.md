# ARS_40x

 ARS_40x package contains a driver for the Continental radar ARS_404 / ARS_408.
 The package also contains a ROS Wrapper for the driver.

## Hardware Setup
Install can-utils to setup radar config
```bash
sudo apt-get install can-utils
candump can0 # Watch the raw data received once the peak CAN bus is installed and connected to Radar
```

### Mode selection

Objects
```bash
cansend can0 200#F8000000089C0000 // Objects detection with all extended properties
```
Clusters
```bash
cansend can0 200#F8000000109C0000 // Clusters detection with all extended properties
```

Filter Selection

Maximum distance of objects detected 30 meters
```bash
cansend can0 202#8C0000012C
```
Minimum value of object RCS -10 dBm2
```bash
cansend can0 202#AE06800FFF
```
Minimum value of objects probability of existence 75%
```bash
cansend can0 202#C600030007
```

## Software Setup
### Dependencies

- [socket_can](https://github.com/Project-MANAS/socket_can)
- [costmap_converter]
- [costmap_converter_msgs]
- [visualization_msgs]
- [perception_msgs]

### Build

```bash
colcon build --symlink-install --packages-up-to ars_40x
```

### Run
1. Enable CAN interface
```bash
sudo ip link set down can0 # If already enabled
sudo ip link set can0 up type can bitrate 500000
```

2. Check if the radar is connected
```bash
candump can0
```

3. Run the driver
```bash
roslaunch ars_40x ars_40x.launch visualize:=true obstacle_array:=true
```
There are arugments available for the launch file:

- **visualize** *(default:"true")* : Launches RViz to display the clusters/obstacles as markers.
- **obstacle_array** *(default:"false")* : Launches ars_40x_obstacle_array node which publishes obstacles as geometry_msgs/Polygon

#### Publications

|Message|Type|Description|Message Box|
|---|---|---|---|
|/radar_status|ars_40x/RadarStatus|Describe the radar configuration|0x201|
|/ars_40x/clusters|ars_40x/ClusterList|Raw clusters data from radar|0x600, 0x701|
|/ars_40x/objects|ars_40x/ObjectList|Raw objects data from radar|0x60A, 0x60B, 0x60C, 0x60D|
|/visualize_clusters|visualization_msgs/MarkerArray|Clusters markers for RViz visualization| - |
|/visualize_objects|visualization_msgs/MarkerArray|Object markers for RViz visualization| - |

#### Subscription

|Message|Type|Description|Message Box|
|---|---|---|---|
|/odom|nav_msgs/Odometry|Velocity and accleration information|0x300, 0x301|


#### [TODO] Services
The following services are available for configuring the radar options available in 0x200

|Services|
|---|
|/set_ctrl_relay_cfg|
|/set_max_distance|
|/set_output_type|
|/set_radar_power|
|/set_rcs_threshold|
|/set_send_ext_info|
|/set_send_quality|
|/set_sensor_id|
|/set_sort_index|
|/set_store_in_nvm|

### [TODO] Change Configuration thru ROS service

> To make the nodes standalone and create a custom can message to change the radar configuration, a ROS service is created to change the radar configuration.

```bash
ros2 service call /set_max_distance perception_msgs/srv/MaxDistance "{max_distance: 160.0}"
```

The command follow the ros2 service call format
```bash
ros2 service call /<service_name> <service_type> "{<service_field>: <value>}"
```

