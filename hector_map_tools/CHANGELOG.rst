^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_map_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2019-10-31)
------------------
* hector_map_tools: Use the FindEigen3.cmake module provided by Eigen
  This patch applies the recommendation from http://wiki.ros.org/jade/Migration and removes the
  dependency from package cmake_modules (unless your installation of Eigen3 does not provide a
  cmake config).
  Same as 1251d9dc20854f48da116eed25780c103a5bd003, but package hector_map_tools was not updated
  back then.
* Contributors: Johannes Meyer

0.3.5 (2016-06-24)
------------------

0.3.4 (2015-11-07)
------------------
* -Fix severe bug when not using square size maps (would results in completely wrong obstacle distances and coordinates)
* Contributors: Stefan Kohlbrecher

0.3.3 (2014-06-15)
------------------

0.3.2 (2014-03-30)
------------------

0.3.1 (2013-10-09)
------------------
* added changelogs

0.3.0 (2013-08-08)
------------------
* catkinized hector_slam
