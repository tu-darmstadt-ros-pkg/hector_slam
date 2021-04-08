^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_geotiff
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2021-04-08)
------------------
* Refactored hector_geotiff dependencies.
* Contributors: Stefan Fabian

0.5.1 (2021-01-15)
------------------
* Fixed "SEVERE WARNING" by pluginloader when killing geotiff node.
  Some minor cleanup.
* Contributors: Stefan Fabian

0.5.0 (2020-12-17)
------------------
* Added missing dependency for Qt5 cmake.
* Moved hector_geotiff launch files to separate package to solve cyclic dependency.
  Clean up for noetic release.
* Qt5 support for hector geotiff on headless systems.
* Updated platform args. Test on robot.
* Experiments with platform argument.
* Renamed depends for (hopefully soon) rosdep compatibility.
* Moved to Qt5.
* Contributors: Stefan Fabian

0.4.1 (2020-05-15)
------------------

0.3.6 (2019-10-31)
------------------
* Update geotiff draw interface to support different shapes
* Contributors: Stefan Kohlbrecher

0.3.5 (2016-06-24)
------------------
* Use the FindEigen3.cmake module provided by Eigen
* hector_geotiff/hector_geotiff_plugins: added possibility to specify Color of robot path in the geotiff file in order to allow multiple color robot paths
* Contributors: Dorothea Koert, Johannes Meyer

0.3.4 (2015-11-07)
------------------
* Removes trailing spaces and fixes indents
* Contributors: YuK_Ota

0.3.3 (2014-06-15)
------------------
* fixed cmake find for eigen in indigo
* added launchfile to restart geotiff node
* Contributors: Alexander Stumpf

0.3.2 (2014-03-30)
------------------
* Add TrajectoryMapWriter to geotiff_mapper.launch per default
* Add arguments to launch files for specifying geotiff file path
* Contributors: Stefan Kohlbrecher

0.3.1 (2013-10-09)
------------------
* added missing install rule for launch files
* moved header files from include/geotiff_writer to include/hector_geotiff
* fixed warnings for deprecated pluginlib method/macro calls
* added changelogs

0.3.0 (2013-08-08)
------------------
* catkinized hector_slam
