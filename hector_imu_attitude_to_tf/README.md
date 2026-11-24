## ROS API

<a id="NodeApi"></a>

### imu_attitude_to_tf_node

`imu_attitude_to_tf_node` is a node that publishes roll/pitch attitude
angles provided by an IMU to tf.

#### Subscribed Topics

- `imu_topic`
  ([sensor_msgs/msg/Imu](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Imu.html)):
  The topic on which IMU data is provided. Has to be remapped to the
  appropiate topic name on the given robot setup.

#### Parameters

- `~base_stabilized_frame` (`string`, default:
  `"base_stabilized_frame"`): The name of the robot base stabilized
  frame that has the x and y axis parallel to the ground
- `~base_frame` (`string`, default: `"base_frame"`): The name of the
  robot base frame rotated around the roll/pitch angles reported by the
  IMU message
