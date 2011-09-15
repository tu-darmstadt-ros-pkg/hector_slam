== Postprocessing Logfile example ==
Download this file:
http://code.google.com/p/tu-darmstadt-ros-pkg/downloads/detail?name=Team_Hector_MappingBox_RoboCup_2011_Rescue_Arena.bag&can=2&q=

Open a new terminal window, type:
rosmake hector_slam

Launch the tutorial launch file provided in the hector_slam_launch package:
roslaunch hector_slam_launch tutorial.launch

Open another terminal, enter the directory of the .bag file and type:
rosbag play Team_Hector_MappingBox_RoboCup_2011_Rescue_Arena.bag --clock

You should now be able see the mapping process in rviz.

