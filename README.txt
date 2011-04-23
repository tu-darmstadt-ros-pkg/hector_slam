=Disclaimer=

The hector_slam stack source code is in the process of getting cleaned up for an official
release. The functionality for processing logfiles below however, can be considered stable
and will continue to work despite changes to code structure and functionality.


=Playing back Datasets=

To process datasets, the following steps have to be taken:
Download a dataset from http://code.google.com/p/tu-darmstadt-ros-pkg/downloads/list 

Open a console, start roscore:
"roscore"

Open another console, set parameter use_sim_time to true:
"rosparam set use_sim_time true"

Launch the hector_scanmatcher with the appropriate launch file:
"roslaunch hector_scanmatcher mapping_box.launch"

Open another console and play back the logfile, for example:
"rosbag play Team_Hector_MappingBox_L101_Building.bag --clock"

The SLAM system is running now, by starting rviz:
"rosrun rviz rviz"
and enabling visualization of the "/map" topic, the mapping process can be visualized.

