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

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
//#include "nav_msgs/GetMap.h"

#include "nav_msgs/Path.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Quaternion.h"

#include "tf/transform_listener.h"
//#include "geometry_msgs/PoseWithCovarianceStamped.h"

//#include <hector_map_tools/HectorMapTools.h>

#include <hector_nav_msgs/GetRobotTrajectory.h>
//#include <hector_nav_msgs/PoseList.h>

#include <tf/tf.h>

using namespace std;


/**
 * @brief Map generation node.
 */
class PathContainer
{
public:
  PathContainer()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param("target_frame_name", p_target_frame_name_, std::string("map"));
    private_nh.param("source_frame_name", p_source_frame_name_, std::string("scanmatcher_frame"));
    private_nh.param("trajectory_update_rate", p_update_rate_, 4.0);


    sys_cmd_sub_ = private_nh.subscribe("syscommand", 1, &PathContainer::sysCmdCallback, this);

    //pose_update_sub_ = n.subscribe("poseupdate", 1, &PathContainer::poseUpdateCallback, this);

    trajectory_provider_service_ = private_nh.advertiseService("robot_trajectory", &PathContainer::trajectoryProviderCallBack, this);

    last_reset_time_ = ros::Time::now();

    update_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_update_rate_), &PathContainer::trajectoryUpdateTimerCallback, this, false);

    pose_source_.pose.orientation.w = 1.0;
    pose_source_.header.frame_id = p_source_frame_name_;

    trajectory_.trajectory.header.frame_id = p_target_frame_name_;

    //dist_lookup_service_ = nh.advertiseService(lookup_service_name, &OccupancyGridContainer::lookupServiceCallback, this);
  }

  void sysCmdCallback(const std_msgs::String& sys_cmd)
  {
    if (sys_cmd.data == "reset"){
      last_reset_time_ = ros::Time::now();
      //path_.poses.poses.clear();
    }
  }

  void trajectoryUpdateTimerCallback(const ros::TimerEvent& event)
  {
    //ROS_INFO("blaa");

    try{
      pose_source_.header.stamp = ros::Time::now();

      //tf::StampedTransform transform;
      geometry_msgs::PoseStamped pose_out;

      //tf_.lookupTransform(p_target_frame_name_,p_source_frame_name_,ros::Time(0), transform);
      ros::Duration dur (0.5);
      tf_.waitForTransform(p_target_frame_name_, pose_source_.header.frame_id, pose_source_.header.stamp, dur);
      tf_.transformPose(p_target_frame_name_, pose_source_, pose_out);

      //tf::poseTFToMsg(transform, geom_pose.pose);


      trajectory_.trajectory.poses.push_back(pose_out);
      trajectory_.trajectory.header.stamp = pose_source_.header.stamp;
      //trajectory_.GetRobotTrajectoryResponse_.trajectory.poses.push_back(transform);
    }
    catch(tf::TransformException e)
    {      
      ROS_ERROR("Transform from %s to %s failed: %s \n", p_target_frame_name_.c_str(), pose_source_.header.frame_id.c_str(), e.what() );
    }
  }

  /*
  void poseUpdateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_in)
  {


    ros::Time now = ros::Time::now();

    //Wait for after a reset till starting to record data.
    if ((now - last_reset_time_).toSec() > 0.5){

      //Wait some time so not too many redundant poses are recorded
      if ( (now - last_pose_save_time_).toSec() > 0.05){
        last_pose_save_time_ = now;
        geometry_msgs::Point32 new_pose;

        new_pose.x = pose_in->pose.pose.position.x;
        new_pose.y = pose_in->pose.pose.position.y;

        tf::Transform transform;
        tf::poseMsgToTF(pose_in->pose.pose,transform);


        btScalar yaw, pitch, roll;
        transform.getBasis().getEulerYPR(yaw, pitch, roll);

        new_pose.z = yaw;

        hector_nav_msgs::PoseList& pose_list (path_.poses);
        std::vector<geometry_msgs::Point32>& pose_vector (pose_list.poses);

        pose_vector.push_back(new_pose);
      }
  }
  */

  bool trajectoryProviderCallBack(hector_nav_msgs::GetRobotTrajectory::Request  &req,
                                  hector_nav_msgs::GetRobotTrajectory::Response &res )
  {
    ROS_INFO("hector_path_provider service called");

    res = trajectory_;

    return true;
  };

  //parameters
  std::string p_target_frame_name_;
  std::string p_source_frame_name_;
  double p_update_rate_;

  // Zero pose used for transformation to target_frame.
  geometry_msgs::PoseStamped pose_source_;

  ros::ServiceServer trajectory_provider_service_;

  ros::Timer update_timer_;


  //ros::Subscriber pose_update_sub_;
  ros::Subscriber sys_cmd_sub_;

  hector_nav_msgs::GetRobotTrajectory::Response trajectory_;

  tf::TransformListener tf_;

  ros::Time last_reset_time_;
  ros::Time last_pose_save_time_;
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_trajectory_server");
  //std::string path_name = "/poseupdate";

  PathContainer pc;

  ros::spin();

  return 0;
}
