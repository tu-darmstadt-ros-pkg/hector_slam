## Overview

This package compresses map data using image transport. Currently, only
a conversion from map to image is available. The opposite direction is
not implemented yet. The code in this package is a proof of concept and
subject to change.

## ROS API

<a id="NodeApi"></a>

### map_to_image_node

Converts map to image data and publishes the images using
image_transport. The conversion is only done when the image data is
requested, so the computational cost of this node running is low when no
images are requested. Both a full map image as well as a tile based map
image that shows only the local area around the robot are made
available. For the latter to work, the robot pose must be available on
the 'pose' topic as described below.

#### Subscribed Topics

- `map`
  ([nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html)):
  The map to transmit
- `pose`
  ([geometry_msgs/msg/PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html)):
  The robot pose given in the map frame

#### Published Topics

- `map_image/full`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  The complete map converted to an image.
- `map_image/tile`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  A tile of the map showing the area around the robot published as an
  image.
