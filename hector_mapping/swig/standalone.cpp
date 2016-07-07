#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan scan;
bool update_scan = false;
void scanCallback(const sensor_msgs::LaserScan& _scan)
{
  static int prev_seq  = 0;
  scan = _scan;
  if ( _scan.header.seq != ( prev_seq + 1) ) {
    std::cerr << "warn !! skipping /scan " << prev_seq << " -> "<< _scan.header.seq << std::endl;
  }
  prev_seq = _scan.header.seq;
  update_scan = true;
}

extern "C" {
#include <HectorSlam_proxy.h>
}

HectorSlamProcessor * slamProcessor;
int lastGetMapUpdateIndex = -100;

#include <boost/thread.hpp>
void saveMapLoop(double _period)
{
  GridMap * gridMap = HectorSlamProcessor_getGridMap_pchectorslam_HectorSlamProcessor_i(slamProcessor, 0);

  ros::Rate r(1.0 / _period);
  while(ros::ok())
  {
    //only update map if it changed
    if (lastGetMapUpdateIndex != GridMapBase_LogOddsCell_getUpdateIndex(gridMap))
    {

      int sizeX = GridMapBase_LogOddsCell_getSizeX(gridMap);
      int sizeY = GridMapBase_LogOddsCell_getSizeY(gridMap);

      int size = sizeX * sizeY;

      // FIX ME, do not have to allocate every loop
      std::vector<int8_t> data;
      data.resize(GridMapBase_LogOddsCell_getSizeX(gridMap) * GridMapBase_LogOddsCell_getSizeY(gridMap));

      //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
      memset(&data[0], 100, sizeof(int8_t) * size);

      for(int i=0; i < size; ++i)
      {
        if(GridMap_isFree_pchectorslam_OccGridMapBase_Sl_LogOddsCell_Sc_GridMapLogOddsFunctions_Sg__i(gridMap, i))
        {
          data[i] = 200; // changed from original variable for better visualization
        }
        else if (GridMap_isOccupied_pchectorslam_OccGridMapBase_Sl_LogOddsCell_Sc_GridMapLogOddsFunctions_Sg__i(gridMap, i))
        {
          data[i] = 0;
        }
      }

      lastGetMapUpdateIndex = GridMapBase_LogOddsCell_getUpdateIndex(gridMap);

      FILE *fp;
      printf("update map.pgm\n");
      fp = fopen("map.pgm", "wb");
      fprintf(fp, "P5\n#\n%d %d\n255\n", GridMapBase_LogOddsCell_getSizeX(gridMap), GridMapBase_LogOddsCell_getSizeY(gridMap));
      fwrite(&data[0], sizeof(int8_t), data.size(), fp);
      fclose(fp);
    }

    r.sleep();
  }
}

#include <Eigen/Core>
int main(int argc, char* argv[]) {
  // invoke ros to subsscribe /scan
  ros::init(argc, argv, "hector_slam_lib_test");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 5, &scanCallback);

  // initialize slamProcessor
  double p_map_resolution_ = 0.050;
  int p_map_size_ = 2048;
  double p_map_start_x_ = 0.5;
  double p_map_start_y_ = 0.5;
  int p_map_multi_res_levels_ = 2;
  double p_update_factor_free_ = 0.4;
  double p_update_factor_occupied_ = 0.9;
  double p_map_update_distance_threshold_ = 0.4;
  double p_map_update_angle_threshold_ = 0.06;

  slamProcessor = new_HectorSlamProcessor_f_i_i_rcEigen_Vector2f_i(p_map_resolution_, p_map_size_, p_map_size_, Eigen_Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_);
  HectorSlamProcessor_setUpdateFactorFree(slamProcessor, p_update_factor_free_);
  HectorSlamProcessor_setUpdateFactorOccupied(slamProcessor, p_update_factor_occupied_);
  HectorSlamProcessor_setMapUpdateMinDistDiff(slamProcessor, p_map_update_distance_threshold_);
  HectorSlamProcessor_setMapUpdateMinAngleDiff(slamProcessor, p_map_update_angle_threshold_);

  // initialize lasersacn Container
  DataPointContainer_Eigen_Vector2f * laserScanContainer;
  laserScanContainer = new_DataPointContainer_Eigen_Vector2f();

  // saveMapLoop runs different thread, to keep main loop (update) runs periodically
  boost::thread* map_save_thread_ = new boost::thread(boost::bind(&saveMapLoop, 0.5));

  while (ros::ok()){
    // when scan is updated
    if ( update_scan ) {
      // update loop
      size_t size = scan.ranges.size();
      float angle = scan.angle_min;
      float angle_increment = scan.angle_increment;
      float range_min = scan.range_min;
      float range_max = scan.range_max;

      //
      float scaleToMap = HectorSlamProcessor_getScaleToMap(slamProcessor);
      DataPointContainer_Eigen_Vector2f_clear(laserScanContainer);
      DataPointContainer_Eigen_Vector2f_setOrigo(laserScanContainer, Eigen_Vector2f(0, 0));

      float maxRangeForContainer = range_max - 0.1f;
      for (size_t i = 0; i < size; ++i)
      {
        float dist = scan.ranges[i];

        if ( (dist > range_min) && (dist < maxRangeForContainer))
        {
          dist *= scaleToMap;
          DataPointContainer_Eigen_Vector2f_add(laserScanContainer, Eigen_Vector2f(cos(angle) * dist, sin(angle) * dist));
        }

        angle += angle_increment;
      }

      HectorSlamProcessor_update_phectorslam_HectorSlamProcessor_rchectorslam_DataPointContainer_Sl_Eigen_Vector2f_Sg__rcEigen_Vector3f(slamProcessor, laserScanContainer, HectorSlamProcessor_getLastScanMatchPose(slamProcessor));

      // show pose
      SwigObj* pose = HectorSlamProcessor_getLastScanMatchPose(slamProcessor);
      std::cerr << "x : " << Eigen_Vector3f_x(pose) << ", y : " << Eigen_Vector3f_y(pose) << ", t : " << Eigen_Vector3f_z(pose) << std::endl;

      //
      update_scan = false;

    }
    ros::spinOnce();
  }
  
}
