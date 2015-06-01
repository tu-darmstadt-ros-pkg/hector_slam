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

#include "nav_msgs/Path.h"
#include "std_msgs/String.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "tf/transform_listener.h"

#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <hector_nav_msgs/GetRecoveryInfo.h>

#include <tf/tf.h>

#include <algorithm>

using namespace std;

bool comparePoseStampedStamps (const geometry_msgs::PoseStamped& t1, const geometry_msgs::PoseStamped& t2) { return (t1.header.stamp < t2.header.stamp); }


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
    private_nh.param("source_frame_name", p_source_frame_name_, std::string("base_link"));
    private_nh.param("trajectory_update_rate", p_trajectory_update_rate_, 4.0);
    private_nh.param("trajectory_publish_rate", p_trajectory_publish_rate_, 0.25);

    waitForTf();

    ros::NodeHandle nh;
    sys_cmd_sub_ = nh.subscribe("syscommand", 1, &PathContainer::sysCmdCallback, this);
    trajectory_pub_ = nh.advertise<nav_msgs::Path>("trajectory",1, true);

    trajectory_provider_service_ = nh.advertiseService("trajectory", &PathContainer::trajectoryProviderCallBack, this);
    recovery_info_provider_service_ = nh.advertiseService("trajectory_recovery_info", &PathContainer::recoveryInfoProviderCallBack, this);

    last_reset_time_ = ros::Time::now();

    update_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_update_rate_), &PathContainer::trajectoryUpdateTimerCallback, this, false);
    publish_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_publish_rate_), &PathContainer::publishTrajectoryTimerCallback, this, false);

    pose_source_.pose.orientation.w = 1.0;
    pose_source_.header.frame_id = p_source_frame_name_;

    trajectory_.trajectory.header.frame_id = p_target_frame_name_;
  }

  void waitForTf()
  {
    ros::Time start = ros::Time::now();
    ROS_INFO("Waiting for tf transform data between frames %s and %s to become available", p_target_frame_name_.c_str(), p_source_frame_name_.c_str() );

    bool transform_successful = false;

    while (!transform_successful){
      transform_successful = tf_.canTransform(p_target_frame_name_, p_source_frame_name_, ros::Time());
      if (transform_successful) break;

      ros::Time now = ros::Time::now();

      if ((now-start).toSec() > 20.0){
        ROS_WARN_ONCE("No transform between frames %s and %s available after %f seconds of waiting. This warning only prints once.", p_target_frame_name_.c_str(), p_source_frame_name_.c_str(), (now-start).toSec());
      }
      
      if (!ros::ok()) return;
      ros::WallDuration(1.0).sleep();
    }

    ros::Time end = ros::Time::now();
    ROS_INFO("Finished waiting for tf, waited %f seconds", (end-start).toSec());
  }


  void sysCmdCallback(const std_msgs::String& sys_cmd)
  {
    if (sys_cmd.data == "reset")
    {
      last_reset_time_ = ros::Time::now();
      trajectory_.trajectory.poses.clear();
      trajectory_.trajectory.header.stamp = ros::Time::now();
    }
  }

  void addCurrentTfPoseToTrajectory()
  {
    pose_source_.header.stamp = ros::Time(0);

    geometry_msgs::PoseStamped pose_out;

    tf_.transformPose(p_target_frame_name_, pose_source_, pose_out);

    if (trajectory_.trajectory.poses.size() != 0){
      //Only add pose to trajectory if it's not already stored
      if (pose_out.header.stamp != trajectory_.trajectory.poses.back().header.stamp){
        trajectory_.trajectory.poses.push_back(pose_out);
      }
    }else{
      trajectory_.trajectory.poses.push_back(pose_out);
    }

    trajectory_.trajectory.header.stamp = pose_out.header.stamp;
  }

  void trajectoryUpdateTimerCallback(const ros::TimerEvent& event)
  {

    try{
      addCurrentTfPoseToTrajectory();
    }catch(tf::TransformException e)
    {
      ROS_WARN("Trajectory Server: Transform from %s to %s failed: %s \n", p_target_frame_name_.c_str(), pose_source_.header.frame_id.c_str(), e.what() );
    }
  }

  void publishTrajectoryTimerCallback(const ros::TimerEvent& event)
  {
    trajectory_pub_.publish(trajectory_.trajectory);
  }

  bool trajectoryProviderCallBack(hector_nav_msgs::GetRobotTrajectory::Request  &req,
                                  hector_nav_msgs::GetRobotTrajectory::Response &res )
  {
    res = getTrajectory();
    return true;
  }

  inline const hector_nav_msgs::GetRobotTrajectoryResponse getTrajectory() const
  {
    return trajectory_;
  }

  bool recoveryInfoProviderCallBack(hector_nav_msgs::GetRecoveryInfo::Request  &req,
                                  hector_nav_msgs::GetRecoveryInfo::Response &res )
  {
    const ros::Time req_time = req.request_time;

    geometry_msgs::PoseStamped tmp;
    tmp.header.stamp = req_time;

    std::vector<geometry_msgs::PoseStamped> const & poses = trajectory_.trajectory.poses;

    if(poses.size() == 0)
    {
        ROS_WARN("Failed to find trajectory leading out of radius %f"
                 " because no poses, i.e. no inverse trajectory, exists.", req.request_radius);
        return false;
    }

    //Find the robot pose in the saved trajectory
    std::vector<geometry_msgs::PoseStamped>::const_iterator it
            = std::lower_bound(poses.begin(), poses.end(), tmp, comparePoseStampedStamps);

    //If we didn't find the robot pose for the desired time, add the current robot pose to trajectory
    if (it == poses.end()){
      addCurrentTfPoseToTrajectory();
      it = poses.end();
      --it;
    }

    std::vector<geometry_msgs::PoseStamped>::const_iterator it_start = it;

    const geometry_msgs::Point& req_coords ((*it).pose.position);

    double dist_sqr_threshold = req.request_radius * req.request_radius;

    double dist_sqr = 0.0;

    //Iterate backwards till the start of the trajectory is reached or we find a pose that's outside the specified radius
    while (it != poses.begin() && dist_sqr < dist_sqr_threshold){
      const geometry_msgs::Point& curr_coords ((*it).pose.position);

      dist_sqr = (req_coords.x - curr_coords.x) * (req_coords.x - curr_coords.x) +
                 (req_coords.y - curr_coords.y) * (req_coords.y - curr_coords.y);

      --it;
    }

    if (dist_sqr < dist_sqr_threshold){
      ROS_INFO("Failed to find trajectory leading out of radius %f", req.request_radius);
      return false;
    }

    std::vector<geometry_msgs::PoseStamped>::const_iterator it_end = it;

    res.req_pose = *it_start;
    res.radius_entry_pose = *it_end;

    std::vector<geometry_msgs::PoseStamped>& traj_out_poses = res.trajectory_radius_entry_pose_to_req_pose.poses;

    res.trajectory_radius_entry_pose_to_req_pose.poses.clear();
    res.trajectory_radius_entry_pose_to_req_pose.header = res.req_pose.header;

    for (std::vector<geometry_msgs::PoseStamped>::const_iterator it_tmp = it_start; it_tmp != it_end; --it_tmp){
      traj_out_poses.push_back(*it_tmp);
    }

    return true;
  }

  //parameters
  std::string p_target_frame_name_;
  std::string p_source_frame_name_;
  double p_trajectory_update_rate_;
  double p_trajectory_publish_rate_;

  // Zero pose used for transformation to target_frame.
  geometry_msgs::PoseStamped pose_source_;

  ros::ServiceServer trajectory_provider_service_;
  ros::ServiceServer recovery_info_provider_service_;

  ros::Timer update_trajectory_timer_;
  ros::Timer publish_trajectory_timer_;


  //ros::Subscriber pose_update_sub_;
  ros::Subscriber sys_cmd_sub_;
  ros::Publisher  trajectory_pub_;

  hector_nav_msgs::GetRobotTrajectory::Response trajectory_;

  tf::TransformListener tf_;

  ros::Time last_reset_time_;
  ros::Time last_pose_save_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_trajectory_server");

  PathContainer pc;

  ros::spin();

  return 0;
}
