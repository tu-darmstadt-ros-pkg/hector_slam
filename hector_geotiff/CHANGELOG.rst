^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_geotiff
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
