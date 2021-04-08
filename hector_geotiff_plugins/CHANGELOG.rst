^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_geotiff_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2021-04-08)
------------------

0.5.1 (2021-01-15)
------------------
* Fixed "SEVERE WARNING" by pluginloader when killing geotiff node.
  Some minor cleanup.
* Contributors: Stefan Fabian

0.5.0 (2020-12-17)
------------------
* Moved hector_geotiff launch files to separate package to solve cyclic dependency.
  Clean up for noetic release.
* Bump CMake version to avoid CMP0048 warning
* Contributors: Marius Schnaubelt, Stefan Fabian

0.4.1 (2020-05-15)
------------------

0.3.6 (2019-10-31)
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
