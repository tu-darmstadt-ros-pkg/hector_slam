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

#include "HectorSlamRos.h"

#include "map/GridMap.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>


#include "sensor_msgs/PointCloud2.h"

#include "tf_conversions/tf_eigen.h"

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

HectorSlamRos::HectorSlamRos()
  : debugInfoProvider(0)
  , hectorDrawings(0)
  , lastGetMapUpdateIndex(-100)
  , tfB_(0)
  , transform_thread_(0)
  , map__publish_thread_(0)
{

  ros::NodeHandle private_nh_("~");

  std::string mapTopic_ = "map";

  private_nh_.param("pub_drawings", p_pub_drawings, false);
  private_nh_.param("pub_debug_output", p_pub_debug_output_, false);
  private_nh_.param("pub_map_odom_transform", p_pub_map_odom_transform_,true);
  private_nh_.param("pub_odometry", p_pub_odometry_,false);

  private_nh_.param("map_resolution", p_map_resolution_, 0.025);
  private_nh_.param("map_size", p_map_size_, 1024);
  private_nh_.param("map_start_x", p_map_start_x_, 0.5);
  private_nh_.param("map_start_y", p_map_start_y_, 0.5);

  private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
  private_nh_.param("sys_msg_topic", p_sys_msg_topic_, std::string("/syscommand"));
  private_nh_.param("pose_update_topic", p_pose_update_topic_, std::string("poseupdate"));

  private_nh_.param("use_tf", p_use_tf_,true);

  private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
  private_nh_.param("map_frame", p_map_frame_, std::string("map"));
  private_nh_.param("odom_frame", p_odom_frame_, std::string("odom"));

  private_nh_.param("output_timing", p_timing_output_,false);


  private_nh_.param("transform_pub_period_", p_transform_pub_period_, 0.05);
  private_nh_.param("map_pub_period_", p_map_pub_period_, 2.0);

  if (p_pub_drawings)
  {
    ROS_INFO("HectorSM publishing debug drawings");
    hectorDrawings = new HectorDrawings();
  }

  if(p_pub_debug_output_)
  {
    ROS_INFO("HectorSM publishing debug info");
    debugInfoProvider = new HectorDebugInfoProvider();
  }

  if(p_pub_odometry_)
  {
    odometryPublisher_ = node_.advertise<nav_msgs::Odometry>("scanmatch_odom", 50);
  }

  slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), hectorDrawings, debugInfoProvider);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  for (int i = 0; i < mapLevels; ++i)
  {
    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0)
    {
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer[i];
    tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
    tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

    if (i == 0)
    {
      tmp.dynamicMapServiceServer_ = node_.advertiseService("dynamic_map", &HectorSlamRos::mapCallback, this);
    }

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));
  }

  ROS_INFO("HectorSM p_base_frame_: %s p_map_frame_: %s p_odom_frame_: %s Scan Topic: %s", p_base_frame_.c_str(), p_map_frame_.c_str(), p_odom_frame_.c_str(), p_scan_topic_.c_str());
  ROS_INFO("HectorSM p_use_tf: %s", p_use_tf_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_transform_pub_period_: %f p_map_pub_period_: %f", p_transform_pub_period_, p_map_pub_period_);


  scanSubscriber_ = node_.subscribe(p_scan_topic_, 10, &HectorSlamRos::scanCallback, this);
  sysMsgSubscriber_ = node_.subscribe(p_sys_msg_topic_, 10, &HectorSlamRos::sysMsgCallback, this);

  poseUpdatePublisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_update_topic_, 1, false);
  posePublisher_ = node_.advertise<geometry_msgs::PoseStamped>("slam_out_pose", 1, false);

  if (p_pub_map_odom_transform_)
  {
    tfB_ = new tf::TransformBroadcaster();
    ROS_ASSERT(tfB_);

    transform_thread_ = new boost::thread(boost::bind(&HectorSlamRos::publishTransformLoop, this, p_transform_pub_period_));
  }

  map__publish_thread_ = new boost::thread(boost::bind(&HectorSlamRos::publishMapLoop, this, p_map_pub_period_));

  map_to_odom_.setIdentity();

  lastMapPublishTime = ros::Time(0,0);
}

HectorSlamRos::~HectorSlamRos()
{
  delete slamProcessor;

  if (hectorDrawings)
    delete hectorDrawings;

  if (debugInfoProvider)
    delete debugInfoProvider;

  if (tfB_)
    delete tfB_;

  if(transform_thread_)
    delete transform_thread_;

  if(map__publish_thread_)
    delete map__publish_thread_;
}

void HectorSlamRos::scanCallback(const sensor_msgs::LaserScan& scan)
{
  if (hectorDrawings)
  {
    hectorDrawings->setTime(scan.header.stamp);
  }

  ros::Time startTime = ros::Time::now();

  if (!p_use_tf_)
  {
    if (rosLaserScanToDataContainer(scan, laserScanContainer,slamProcessor->getScaleToMap()))
    {
      slamProcessor->update(laserScanContainer,slamProcessor->getLastScanMatchPose());
    }
  }
  else
  {

    sensor_msgs::PointCloud pointCloud;

    ros::Duration dur (10.0);

    //if (tf_.waitForTransform(p_base_frame_,scan.header.stamp,scan.header.frame_id,scan.header.stamp,p_base_frame_,dur)){

    if (tf_.waitForTransform(p_base_frame_,scan.header.frame_id, scan.header.stamp,dur))
    {

      tf::StampedTransform laserTransform;      
      tf_.lookupTransform(p_base_frame_,scan.header.frame_id, scan.header.stamp, laserTransform);

      //projector_.transformLaserScanToPointCloud(p_base_frame_ ,scan, pointCloud,tf_);
      projector_.projectLaser(scan, pointCloud,30.0);

      if(rosPointCloudToDataContainer(pointCloud, laserTransform, laserScanContainer, slamProcessor->getScaleToMap()))
      {
        slamProcessor->update(laserScanContainer,slamProcessor->getLastScanMatchPose());
      }

    }
    else
    {
      ROS_INFO("waitForTransform timed out");
    }

  }

  if (p_timing_output_)
  {
    ros::Duration duration = ros::Time::now() - startTime;
    ROS_INFO("HectorSLAM Iter took: %f milliseconds", duration.toSec()*1000.0f );
  }

  const Eigen::Vector3f& slamPose(slamProcessor->getLastScanMatchPose());
  const Eigen::Matrix3f& slamCov(slamProcessor->getLastScanMatchCovariance());

  std_msgs::Header header;
  header.stamp = scan.header.stamp;
  header.seq = scan.header.seq;
  header.frame_id = "map";

  geometry_msgs::Pose pose;
  pose.position.x = slamPose.x();
  pose.position.y = slamPose.y();

  pose.orientation.w = cos(slamPose.z()*0.5f);
  pose.orientation.z = sin(slamPose.z()*0.5f);

  geometry_msgs::PoseStamped stampedPose;
  stampedPose.header = header;
  stampedPose.pose = pose;

  geometry_msgs::PoseWithCovarianceStamped covPose;
  covPose.header = header;
  covPose.pose.pose = pose;

  boost::array<double, 36>& cov(covPose.pose.covariance);

  cov[0] = static_cast<double>(slamCov(0,0));
  cov[7] = static_cast<double>(slamCov(1,1));
  cov[21] = static_cast<double>(slamCov(2,2));

  double xyC = static_cast<double>(slamCov(0,1));
  cov[1] = xyC;
  cov[6] = xyC;

  double xaC = static_cast<double>(slamCov(0,2));
  cov[3] = xaC;
  cov[18] = xaC;

  double yaC = static_cast<double>(slamCov(1,2));
  cov[9] = yaC;
  cov[19] = yaC;

  poseUpdatePublisher_.publish(covPose);
  posePublisher_.publish(stampedPose);

  if(p_pub_odometry_)
  {
    nav_msgs::Odometry tmp;
    tmp.pose = covPose.pose;

    tmp.header = header;
    odometryPublisher_.publish(tmp);
  }

  tf::Stamped<tf::Pose> odom_to_map;

  if (p_use_tf_ && p_pub_map_odom_transform_)
  {
    try
    {
      tf_.transformPose(p_odom_frame_,tf::Stamped<tf::Pose> (btTransform(tf::createQuaternionFromRPY(0.0, 0.0, static_cast<double>(slamPose.z())),
                                                                       btVector3(static_cast<double>(slamPose.x()), static_cast<double>(slamPose.y()), 0.0)).inverse(),
                                                           scan.header.stamp, p_base_frame_),odom_to_map);
    }
    catch(tf::TransformException e)
    {
      ROS_ERROR("Transform from %s to %s failed\n", p_base_frame_.c_str(), p_odom_frame_.c_str());
      odom_to_map.setIdentity();
    }

    map_to_odom_mutex_.lock();
    map_to_odom_ = tf::Transform(tf::Quaternion( odom_to_map.getRotation() ),
                                 tf::Point(      odom_to_map.getOrigin() ) ).inverse();
    map_to_odom_mutex_.unlock();
  }

  //p_map_pub_period_ = ros::Duration(10.0);
  //p_map_pub_period_ = ros::Duration(0.0);
  /*
  if ( (scan.header.stamp - lastMapPublishTime) > p_map_pub_period_){
    publishMap(slamProcessor->getGridMap(),scan.header.stamp);
    lastMapPublishTime = scan.header.stamp;
  }
  */
}

void HectorSlamRos::sysMsgCallback(const std_msgs::String& string)
{
  ROS_INFO("HectorSM sysMsgCallback, msg contents: %s", string.data.c_str());

  if (string.data == "reset")
  {
    ROS_INFO("HectorSM reset");
    slamProcessor->reset();
  }
}

bool HectorSlamRos::mapCallback(nav_msgs::GetMap::Request  &req,
                                nav_msgs::GetMap::Response &res)
{
  res = mapPubContainer[0].map_;
  return true;
}

void HectorSlamRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex)
{
  nav_msgs::GetMap::Response& map_ (mapPublisher.map_);

  //only update map if it changed
  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
  {

    if (mapMutex)
    {
      mapMutex->lockMap();
    }

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    for(int i=0; i < size; ++i)
    {
      if(gridMap.isFree(i))
      {
        data[i] = 0;
      }
      else if (gridMap.isOccupied(i))
      {
        data[i] = 100;
      }
      else
      {
        data[i] = -1;
      }
    }

    lastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex)
    {
      mapMutex->unlockMap();
    }
  }

  map_.map.header.stamp = timestamp;

  mapPublisher.mapMetadataPublisher_.publish(map_.map.info);
  mapPublisher.mapPublisher_.publish(map_.map);
}

bool HectorSlamRos::rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  unsigned int size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (unsigned int i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }

  return true;
}

bool HectorSlamRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  unsigned int size = pointCloud.points.size();
  //ROS_INFO("size: %d", size);

  dataContainer.clear();

  tf::Vector3 laserPos (laserTransform*tf::Vector3(0.0, 0.0, 0.0));
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y())*scaleToMap);

  for (unsigned int i = 0; i < size; ++i)
  {

    const geometry_msgs::Point32& currPoint(pointCloud.points[i]);

    tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

    if (pointPosBaseFrame.z() > -1.0f && pointPosBaseFrame.z() < 1.0f)
    {
      dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
    }
  }

  return true;
}

void HectorSlamRos::setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
{
  Eigen::Vector2f mapOrigin (gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.cwise() -= gridMap.getCellLength()*0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

void HectorSlamRos::publishTransformLoop(double p_transform_pub_period_)
{
  if(p_transform_pub_period_ == 0)
    return;

  ros::Rate r(1.0 / p_transform_pub_period_);
  while(ros::ok())
  {
    publishTransform();
    r.sleep();
  }
}

void HectorSlamRos::publishMapLoop(double p_transform_pub_period_)
{
  if(p_transform_pub_period_ == 0)
    return;

  ros::Rate r(1.0 / p_transform_pub_period_);
  while(ros::ok())
  {

    ros::Time mapTime (ros::Time::now());
    //publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
    //publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
    publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    r.sleep();
  }
}

void HectorSlamRos::publishTransform()
{
  map_to_odom_mutex_.lock();
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, ros::Time::now(), p_map_frame_, p_odom_frame_));
  map_to_odom_mutex_.unlock();
}

