^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_trajectory_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2019-10-31)
------------------

0.3.5 (2016-06-24)
------------------
* Changed from ros::WallTime to ros::Time in trajectory server
* hector_trajectory_server: removed bug leading to potential infinite loop
* Contributors: Andreas Lindahl Flåten, Paul Manns

0.3.4 (2015-11-07)
------------------

0.3.3 (2014-06-15)
------------------

0.3.2 (2014-03-30)
------------------
* wait based on WallTime and check ros::ok() in waitForTf()
* Print out tf availability warning only once.
* Wait for tf become available to suppress warnings on startup
* Create a warning instead of an error for TransformExceptions
* Contributors: Johannes Meyer, Stefan Kohlbrecher, wachaja

0.3.1 (2013-10-09)
------------------
* added changelogs
* added cmake dependencies to catkin exported targets to force message targets to be built before any cpp target using them

0.3.0 (2013-08-08)
------------------
* catkinized hector_slam
