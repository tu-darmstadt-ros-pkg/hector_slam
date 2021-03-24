//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_MAPPING_ROS_H__
#define HECTOR_MAPPING_ROS_H__

#include "ros/ros.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include "sensor_msgs/LaserScan.h"
#include <std_msgs/String.h>

#include <hector_mapping/ResetMapping.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "laser_geometry/laser_geometry.h"
#include "nav_msgs/GetMap.h"

#include "slam_main/HectorSlamProcessor.h"

#include "scan/DataPointContainer.h"
#include "util/MapLockerInterface.h"

#include <boost/thread.hpp>

#include "PoseInfoContainer.h"


class HectorDrawings;
class HectorDebugInfoProvider;

class MapPublisherContainer
{
public:
  ros::Publisher mapPublisher_;
  ros::Publisher mapMetadataPublisher_;
  nav_msgs::GetMap::Response map_;
  ros::ServiceServer dynamicMapServiceServer_;
};

class HectorMappingRos
{
public:
  HectorMappingRos();
  ~HectorMappingRos();


  void scanCallback(const sensor_msgs::LaserScan& scan);
  void sysMsgCallback(const std_msgs::String& string);

  bool mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
  bool resetMapCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
  bool restartHectorCallback(hector_mapping::ResetMapping::Request  &req, hector_mapping::ResetMapping::Response &res);
  bool pauseMapCallback(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res);

  void publishMap(MapPublisherContainer& map_, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex = 0);

  void rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap);
  void rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap);

  void setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap);

  void publishTransformLoop(double p_transform_pub_period_);
  void publishMapLoop(double p_map_pub_period_);
  void publishTransform();

  void staticMapCallback(const nav_msgs::OccupancyGrid& map);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  // Internal mapping management functions
  void toggleMappingPause(bool pause);
  void resetPose(const geometry_msgs::Pose &pose);

  /*
  void setStaticMapData(const nav_msgs::OccupancyGrid& map);
  */
protected:

  HectorDebugInfoProvider* debugInfoProvider;
  HectorDrawings* hectorDrawings;

  int lastGetMapUpdateIndex;

  ros::NodeHandle node_;

  ros::Subscriber scanSubscriber_;
  ros::Subscriber sysMsgSubscriber_;

  ros::Subscriber mapSubscriber_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_sub_;
  tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_filter_;

  ros::Publisher posePublisher_;
  ros::Publisher poseUpdatePublisher_;
  ros::Publisher twistUpdatePublisher_;
  ros::Publisher odometryPublisher_;
  ros::Publisher scan_point_cloud_publisher_;

  ros::ServiceServer reset_map_service_;
  ros::ServiceServer restart_hector_service_;
  ros::ServiceServer toggle_scan_processing_service_;

  std::vector<MapPublisherContainer> mapPubContainer;

  tf::TransformListener tf_;
  tf::TransformBroadcaster* tfB_;

  laser_geometry::LaserProjection projector_;

  tf::Transform map_to_odom_;

  boost::thread* map__publish_thread_;

  hectorslam::HectorSlamProcessor* slamProcessor;
  hectorslam::DataContainer laserScanContainer;

  PoseInfoContainer poseInfoContainer_;

  sensor_msgs::PointCloud laser_point_cloud_;

  ros::Time lastMapPublishTime;
  ros::Time lastScanTime;
  Eigen::Vector3f lastSlamPose;

  bool initial_pose_set_;
  Eigen::Vector3f initial_pose_;

  bool pause_scan_processing_;

  //-----------------------------------------------------------
  // Parameters

  std::string p_base_frame_;
  std::string p_map_frame_;
  std::string p_odom_frame_;

  //Parameters related to publishing the scanmatcher pose directly via tf
  bool p_pub_map_scanmatch_transform_;
  std::string p_tf_map_scanmatch_transform_frame_name_;

  std::string p_scan_topic_;
  std::string p_sys_msg_topic_;

  std::string p_pose_update_topic_;
  std::string p_twist_update_topic_;

  bool p_pub_drawings;
  bool p_pub_debug_output_;
  bool p_pub_map_odom_transform_;
  bool p_pub_odometry_;
  bool p_advertise_map_service_;
  int p_scan_subscriber_queue_size_;

  double p_update_factor_free_;
  double p_update_factor_occupied_;
  double p_map_update_distance_threshold_;
  double p_map_update_angle_threshold_;

  double p_map_resolution_;
  int p_map_size_;
  double p_map_start_x_;
  double p_map_start_y_;
  int p_map_multi_res_levels_;

  double p_map_pub_period_;

  bool p_use_tf_scan_transformation_;
  bool p_use_tf_pose_start_estimate_;
  bool p_map_with_known_poses_;
  bool p_timing_output_;

  float p_sqr_laser_min_dist_;
  float p_sqr_laser_max_dist_;
  float p_laser_z_min_value_;
  float p_laser_z_max_value_;
};

#endif
