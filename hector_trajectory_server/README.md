## Documentation

This package provides a node that saves [tf](https://index.ros.org/p/tf)
based trajectory data given a target and and source frame. The
trajectory is saved internally as a
[nav_msgs/msg/Path](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Path.html)
and can be obtained using a service or topic. Internally, a
tf::Transformlistener listens to [tf](https://index.ros.org/p/tf),
performs the necessary transformations and pushes resulting poses to the
saved trajectory. The update and publish rates can be modified by
parameters as described below.

## ROS API

<a id="NodeApi"></a>

### hector_trajectory_server

`hector_trajectory_server` is a node that saves trajectory data and
makes it available using the ROS API described below.

#### Subscribed Topics

- `syscommand`
  ([std_msgs/msg/String](https://docs.ros.org/en/rolling/p/std_msgs/msg/String.html)):
  System command. If the string equals "reset" the saved trajectory is
  cleared.

#### Published Topics

- `trajectory`
  ([nav_msgs/msg/Path](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Path.html)):
  Get the traveled trajectory from this latched topic.

#### Services

- `trajectory`
  ([hector_nav_msgs/srv/GetRobotTrajectory](https://docs.ros.org/en/rolling/p/hector_nav_msgs/srv/GetRobotTrajectory.html)):
  Call this service to get the traveled trajectory.

#### Parameters

- `~target_frame_name` (`string`, default: `"map"`): The name of the
  [tf](https://index.ros.org/p/tf) target frame
- `~source_frame_name` (`string`, default: `"base_link"`): The name
  of the [tf](https://index.ros.org/p/tf) source frame
- `~trajectory_update_rate` (`double`, default: `4.0`): The update rate
  \[hz\] for the saved trajectory
- `~trajectory_publish_rate` (`double`, default: `0.25`): The publish
  rate \[hz\] for the trajectory published on the "trajectory" topic.
