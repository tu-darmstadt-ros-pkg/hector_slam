## Overview

An example video can be seen here:

[Youtube](https://www.youtube.com/watch?v=Cfq3s4-H2S4)

Further video examples are available in this playlist:
<http://www.youtube.com/playlist?list=PL0E462904E5D35E29>

Details can also be found in this paper: 

```
@INPROCEEDINGS{KohlbrecherMeyerStrykKlingaufFlexibleSlamSystem2011,
  author = {S. Kohlbrecher and J. Meyer and O. von Stryk and U. Klingauf},
  title = {A Flexible and Scalable SLAM System with Full 3D Motion Estimation},
  year = {2011},
  month = {November},
  booktitle = {Proc. IEEE International Symposium on Safety, Security and Rescue Robotics (SSRR)},
  organization = {IEEE},
}
```


available
[here](http://www.sim.tu-darmstadt.de/publ/download/2011_SSRR_KohlbrecherMeyerStrykKlingauf_Flexible_SLAM_System.pdf)
.

## Hardware Requirements

To use `hector_mapping`, you need a source of
[sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)
data (for example a Hokuyo UTM-30LX LIDAR or bagfiles). The node uses
[tf](https://index.ros.org/p/tf) for transformation of scan data, so the
LIDAR does not have to be fixed related to the specified base frame.
Odometry data is not needed.

## ROS API

<a id="NodeApi"></a>

### hector_mapping

`hector_mapping` is a node for LIDAR based SLAM with no odometry and
low computational resources. For simplicity, the ROS API detailed below
provides information about the commonly used options from a user
perspective, but not all options that are available for debugging
purposes.

#### Subscribed Topics

- `scan`
  ([sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)):
  The laser scan used by the SLAM system.
- `syscommand`
  ([std_msgs/msg/String](https://docs.ros.org/en/rolling/p/std_msgs/msg/String.html)):
  System command. If the string equals "reset" the map and robot pose
  are reset to their inital state.

#### Published Topics

- `map_metadata`
  ([nav_msgs/msg/MapMetaData](https://docs.ros.org/en/rolling/p/nav_msgs/msg/MapMetaData.html)):
  Get the map data from this topic, which is latched, and updated
  periodically.
- `map`
  ([nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html)):
  Get the map data from this topic, which is latched, and updated
  periodically
- `slam_out_pose`
  ([geometry_msgs/msg/PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html)):
  The estimated robot pose without covariance
- `poseupdate`
  ([geometry_msgs/msg/PoseWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseWithCovarianceStamped.html)):
  The estimated robot pose with an gaussian estimate of uncertainty

#### Services

- `dynamic_map`
  ([nav_msgs/srv/GetMap](https://docs.ros.org/en/rolling/p/nav_msgs/srv/GetMap.html)):
  Call this service to get the map data.
- `reset_map`
  ([std_srvs/srv/Trigger](https://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html)):
  Call this service to reset the map, and hector will start a whole new
  map from scratch. Notice that this doesn't restart the robot's pose,
  and it will restart from the last recorded pose.
- `pause_mapping`
  ([std_srvs/srv/SetBool](https://docs.ros.org/en/rolling/p/std_srvs/srv/SetBool.html)):
  Call this service to stop/start processing laser scans.
- `restart_mapping_with_new_pose`
  ([hector_mapping/srv/ResetMapping](https://docs.ros.org/en/rolling/p/hector_mapping/srv/ResetMapping.html)):
  Call this service to reset the map, the robot's pose, and resume
  mapping (if paused)

#### Parameters

- `~base_frame` (`string`, default: `base_link`): The name of the base
  frame of the robot. This is the frame used for localization and for
  transformation of laser scan data.
- `~map_frame` (`string`, default: `map_link`): The name of the map
  frame.
- `~odom_frame` (`string`, default: `odom`): The name of the odom frame.
- `~map_resolution` (`double`, default: `0.025`): The map resolution
  \[m\]. This is the length of a grid cell edge.
- `~map_size` (`int`, default: `1024`): The size \[number of cells per
  axis\] of the map. The map is square and has (map_size \* map_size)
  grid cells.
- `~map_start_x` (`double`, default: `0.5`): Location of the origin
  \[0.0, 1.0\] of the /map frame on the x axis relative to the grid map.
  0.5 is in the middle.
- `~map_start_y` (`double`, default: `0.5`): Location of the origin
  \[0.0, 1.0\] of the /map frame on the y axis relative to the grid map.
  0.5 is in the middle.
- `~map_update_distance_thresh` (`double`, default: `0.4`): Threshold
  for performing map updates \[m\]. The platform has to travel this far
  in meters or experience an angular change as described by the
  map_update_angle_thresh parameter since the last update before a map
  update happens.
- `~map_update_angle_thresh` (`double`, default: `0.9`): Threshold for
  performing map updates \[rad\]. The platform has to experience an
  angular change as described by this parameter of travel as far as
  specified by the map_update_distance_thresh parameter since the last
  update before a map update happens.
- `~map_pub_period` (`double`, default: `2.0`): The map publish period
  \[s\].
- `~map_multi_res_levels` (`int`, default: `3`): The number of map
  multi-resolution grid levels.
- `~update_factor_free` (`double`, default: `0.4`): The map update
  modifier for updates of free cells in the range \[0.0, 1.0\]. A value
  of 0.5 means no change.
- `~update_factor_occupied` (`double`, default: `0.9`): The map update
  modifier for updates of occupied cells in the range \[0.0, 1.0\]. A
  value of 0.5 means no change.
- `~laser_min_dist` (`double`, default: `0.4`): The minimum distance
  \[m\] for laser scan endpoints to be used by the system. Scan
  endpoints closer than this value are ignored.
- `~laser_max_dist` (`double`, default: `30.0`): The maximum distance
  \[m\] for laser scan endpoints to be used by the system. Scan
  endpoints farther away than this value are ignored.
- `~laser_z_min_value` (`double`, default: `-1.0`): The minimum height
  \[m\] relative to the laser scanner frame for laser scan endpoints to
  be used by the system. Scan endpoints lower than this value are
  ignored.
- `~laser_z_max_value` (`double`, default: `1.0`): The maximum height
  \[m\] relative to the laser scanner frame for laser scan endpoints to
  be used by the system. Scan endpoints higher than this value are
  ignored.
- `~pub_map_odom_transform` (`bool`, default: `true`): Determine if the
  map->odom transform should be published by the system.
- `~output_timing` (`bool`, default: `false`): Output timing information
  for processing of every laser scan via ROS_INFO.
- `~scan_subscriber_queue_size` (`int`, default: `5`): The queue size of
  the scan subscriber. This should be set to high values (for
  example 50) if logfiles are played back to hector_mapping at faster
  than realtime speeds.
- `~pub_map_scanmatch_transform` (`bool`, default: `true`): Determines
  if the scanmatcher to map transform should be published to tf. The
  frame name is determined by the
  'tf_map_scanmatch_transform_frame_name' parameter.
- `~tf_map_scanmatch_transform_frame_name` (`string`, default:
  `scanmatcher_frame`): The frame name when publishing the scanmatcher
  to map transform as described in the preceding parameter.

#### Required tf Transforms

- `<the frame attached to incoming scans>` → `base_frame`: usually a
  fixed value, broadcast periodically by a
  [robot_state_publisher](https://index.ros.org/p/robot_state_publisher),
  or a `tf` [static_transform_publisher](https://index.ros.org/p/tf).

#### Provided tf Transforms

- `map` → `odom`: the current estimate of the robot's pose within the
  map frame (only provided if parameter "pub_map_odom_transform" is
  true).
