//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, Johannes Meyer TU Darmstadt
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

#include <stdio.h>
#include <stdlib.h>
//#include <libgen.h>
//#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include "nav_msgs/GetMap.h"

#include "hector_marker_drawing/HectorDrawings.h"
#include "hector_map_tools/HectorMapTools.h"

#include "hector_nav_msgs/GetDistanceToObstacle.h"
#include "hector_nav_msgs/GetSearchPosition.h"


class OccupancyGridContainer
{
public:
  OccupancyGridContainer(std::string sub_topic, std::string prefix, ros::NodeHandle& nh, HectorDrawings* drawing_provider, tf::TransformListener* tf_)
    : drawing_provider_(drawing_provider)
    , tf_(tf_)
  {

    std::string service_name = "map";
    map_service_ = nh.advertiseService(service_name, &OccupancyGridContainer::mapServiceCallback, this);

    ros::NodeHandle pnh("~");
    std::string lookup_service_name = "get_distance_to_obstacle";
    dist_lookup_service_ = pnh.advertiseService(lookup_service_name, &OccupancyGridContainer::lookupServiceCallback, this);

    std::string get_search_pos_service_name = "get_search_position";
    get_search_pos_service_ = pnh.advertiseService(get_search_pos_service_name, &OccupancyGridContainer::getSearchPosServiceCallback, this);

    map_sub_ = nh.subscribe("map", 1, &OccupancyGridContainer::mapCallback, this);
  }

  ~OccupancyGridContainer()
  {}

  bool mapServiceCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res )
  {
    ROS_INFO("hector_map_server map service called");

    if (!map_ptr_){
      ROS_INFO("map_server has no map yet, no map service available");
      return false;
    }

    res.map = *map_ptr_;

    return true;
  }

  bool lookupServiceCallback(hector_nav_msgs::GetDistanceToObstacle::Request  &req,
                          hector_nav_msgs::GetDistanceToObstacle::Response &res )
  {
    //ROS_INFO("hector_map_server lookup service called");


    if (!map_ptr_){
      ROS_INFO("map_server has no map yet, no lookup service available");
      return false;
    }

    tf::StampedTransform stamped_pose;



    try{
      tf_->waitForTransform(map_ptr_->header.frame_id,req.point.header.frame_id, req.point.header.stamp, ros::Duration(1.0));
      tf_->lookupTransform(map_ptr_->header.frame_id, req.point.header.frame_id, req.point.header.stamp, stamped_pose);

      tf::Point v2_tf;
      tf::pointMsgToTF(req.point.point,v2_tf);

      tf::Vector3 v1 = stamped_pose * tf::Vector3(0.0, 0.0, 0.0);
      tf::Vector3 v2 = stamped_pose * v2_tf;
      tf::Vector3 diff = v2 - v1;
      v2 = v1 + diff / tf::Vector3(diff.x(), diff.y(), 0.0).length() * 5.0f;

      Eigen::Vector2f start(v1.x(),v1.y());
      Eigen::Vector2f end(v2.x(),v2.y());

      Eigen::Vector2f hit_world;
      //float dist = dist_meas_.getDist(start,end, &hit_world);
      float dist = dist_meas_.getDist(start,end,&hit_world);

      if (dist >=0.0f){
        tf::Vector3 diff (v2-v1);

        float angle = diff.angle(tf::Vector3(diff.x(),diff.y(),0.0f));

        res.distance = dist/cos(angle);

      }else{
        res.distance = -1.0f;
      }

      //debug drawing
      if (false){

        float cube_scale = map_ptr_->info.resolution;
        drawing_provider_->setColor(1.0, 0.0, 0.0);
        drawing_provider_->setScale(static_cast<double>(cube_scale));

        drawing_provider_->drawPoint(start);

        drawing_provider_->setColor(0.0, 1.0, 0.0);
        drawing_provider_->drawPoint(end);

        if (dist >= 0.0f){
          drawing_provider_->setColor(0.0, 0.0, 1.0);
          drawing_provider_->drawPoint(hit_world);
        }

        drawing_provider_->sendAndResetData();
      }

      return true;

    }
    catch(tf::TransformException e)
    {
      ROS_ERROR("Transform failed in lookup distance service call: %s",e.what());
    }

    return false;
  }

  bool getSearchPosServiceCallback(hector_nav_msgs::GetSearchPosition::Request  &req,
                                   hector_nav_msgs::GetSearchPosition::Response &res )
  {
    if (!map_ptr_){
      ROS_INFO("map_server has no map yet, no get best search pos service available");
      return false;
    }

    try{

      tf::Stamped<tf::Pose> ooi_pose, transformed_pose, search_position;
      tf::poseStampedMsgToTF(req.ooi_pose, ooi_pose);

      tf_->waitForTransform(map_ptr_->header.frame_id, req.ooi_pose.header.frame_id, req.ooi_pose.header.stamp, ros::Duration(1.0));
      tf_->transformPose(map_ptr_->header.frame_id, ooi_pose, transformed_pose);

      tf::Vector3 direction(-req.distance, 0.0, 0.0);
      search_position = transformed_pose;
      search_position.setOrigin(transformed_pose.getOrigin() + transformed_pose.getBasis() * direction);

      tf::poseStampedTFToMsg(search_position, res.search_pose);

      return true;

//      //tf::Point v2_tf;
//      //tf::pointMsgToTF(req.point.point,v2_tf);

//      tf::Vector3 v1 = stamped_pose * tf::Vector3(0.0, 0.0, 0.0);

//      //warning: 3D!
//      tf::Vector3 v2 = stamped_pose * tf::Vector3(-1.0, 0.0, 0.0);

//      tf::Vector3 dir = v2-v1;

//      Eigen::Vector2f dir_2d (dir.x(), dir.y());

//      dir_2d.normalize();

//      Eigen::Vector2f searchPos (Eigen::Vector2f(v1.x(),v1.y()) + (dir_2d*0.5f));

//      //copy original pose message but set translation
//      res.search_pose.pose = ooi_pose.pose;

//      res.search_pose.pose.position.x = searchPos.x();
//      res.search_pose.pose.position.y = searchPos.y();

      //return true;


      //Eigen::Vector2f ooi_pos(v1.x(),v1.y());
      //Eigen::Vector2f sample_point_pos(v2.x(),v2.y());


      //float dist_from_target = dist_meas_.getDist(ooi_pos,sample_point_pos);
      //float dist_from_sample_point = dist_meas_.getDist(sample_point_pos, ooi_pos);



/*
      if (dist >=0.0f){
        tf::Vector3 diff (v2-v1);

        float angle = diff.angle(tf::Vector3(diff.x(),diff.y(),0.0f));

        res.distance = dist/cos(angle);


        //debug drawing
        if (true){

          float cube_scale = map_ptr_->info.resolution;
          drawing_provider_->setColor(1.0, 0.0, 0.0);
          drawing_provider_->setScale(static_cast<double>(cube_scale));

          drawing_provider_->drawPoint(start);
          drawing_provider_->drawPoint(end);

          if (dist >= 0.0f){
            drawing_provider_->setColor(0.0, 0.0, 1.0);
            drawing_provider_->drawPoint(hit_world);
          }

          drawing_provider_->sendAndResetData();
        }

      }
      */
      //return true;

    }    catch(tf::TransformException e){
      ROS_ERROR("Transform failed in getSearchPosition service call: %s",e.what());
    }

    return false;
  }

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
  {
    map_ptr_ = map;

    dist_meas_.setMap(map_ptr_);
  }

  //Services
  ros::ServiceServer map_service_;
  ros::ServiceServer dist_lookup_service_;
  ros::ServiceServer get_search_pos_service_;

  //Subscriber
  ros::Subscriber map_sub_;

  HectorMapTools::DistanceMeasurementProvider dist_meas_;

  HectorDrawings* drawing_provider_;
  tf::TransformListener* tf_;

  //nav_msgs::MapMetaData meta_data_message_;
  nav_msgs::GetMap::Response map_resp_;

  nav_msgs::OccupancyGridConstPtr map_ptr_;
};

class HectorMapServer
{
  public:
    /** Trivial constructor */
    HectorMapServer(ros::NodeHandle& private_nh)
    {
      std::string frame_id;

      hector_drawings_ = new HectorDrawings();
      hector_drawings_->setNamespace("map_server");

      mapContainer = new OccupancyGridContainer("map", "" ,private_nh, hector_drawings_,&tf_);
    }

    ~HectorMapServer()
    {
      delete mapContainer;


      delete hector_drawings_;
    }


public:
    OccupancyGridContainer* mapContainer;
private:

    HectorDrawings* hector_drawings_;
    tf::TransformListener tf_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_map_server");
  ros::NodeHandle nh;

  HectorMapServer ms(nh);

  ros::spin();

  return 0;
}

