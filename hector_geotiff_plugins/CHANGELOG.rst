^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_geotiff_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2019-10-31)
------------------

0.4.1 (2020-05-15)
------------------

0.3.5 (2016-06-24)
------------------
* hector_geotiff/hector_geotiff_plugins: added possibility to specify Color of robot path in the geotiff file in order to allow multiple color robot paths
* Contributors: Dorothea Koert

0.3.4 (2015-11-07)
------------------

0.3.3 (2014-06-15)
------------------

0.3.2 (2014-03-30)
------------------

0.3.1 (2013-10-09)
------------------
* readded PLUGINLIB_DECLARE_CLASS macro for fuerte compatibility
* use hector_geotiff_plugins/TrajectoryMapWriter as legacy lookup name for the trajectory geotiff plugin to not break old launch files
* fixed warnings for deprecated pluginlib method/macro calls
* added changelogs
* added cmake dependencies to catkin exported targets to force message targets to be built before any cpp target using them

0.3.0 (2013-08-08)
------------------
* catkinized hector_slam
