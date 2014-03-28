//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
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


#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

std::string p_base_stabilized_frame_;
std::string p_base_frame_;
tf::TransformBroadcaster* tfB_;
tf::StampedTransform transform_;
tf::Quaternion tmp_;

sensor_msgs::ImuConstPtr last_imu_msg_;
sensor_msgs::Imu fused_imu_msg_;
geometry_msgs::PoseStampedConstPtr last_pose_msg_;

ros::Publisher fused_imu_publisher_;

#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void imuMsgCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  //last_imu_msg_ = imu_msg;

  if (last_pose_msg_ != 0){
    tf::quaternionMsgToTF(imu_msg->orientation, tmp_);

    tfScalar imu_yaw, imu_pitch, imu_roll;
    tf::Matrix3x3(tmp_).getRPY(imu_roll, imu_pitch, imu_yaw);

    tf::quaternionMsgToTF(last_pose_msg_->pose.orientation, tmp_);

    tfScalar pose_yaw, pose_pitch, pose_roll;
    tf::Matrix3x3(tmp_).getRPY(pose_roll, pose_pitch, pose_yaw);

    tf::Quaternion tmp;
    tmp.setRPY(imu_roll, imu_pitch, pose_yaw);

    fused_imu_msg_.header.stamp = imu_msg->header.stamp;

    fused_imu_msg_.orientation.x = tmp.getX();
    fused_imu_msg_.orientation.y = tmp.getY();
    fused_imu_msg_.orientation.z = tmp.getZ();
    fused_imu_msg_.orientation.w = tmp.getW();

    fused_imu_publisher_.publish(fused_imu_msg_);
  }
}

void poseMsgCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  last_pose_msg_ = pose_msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pn.param("base_stabilized_frame", p_base_stabilized_frame_, std::string("base_stabilized"));
  pn.param("base_frame", p_base_frame_, std::string("base_link"));

  fused_imu_msg_.header.frame_id = p_base_stabilized_frame_;

  fused_imu_publisher_ = n.advertise<sensor_msgs::Imu>("/fused_imu",1,false);

  ros::Subscriber imu_subscriber = n.subscribe("/imu", 10, imuMsgCallback);
  ros::Subscriber pose_subscriber = n.subscribe("/pose", 10, poseMsgCallback);

  ros::spin();

  return 0;
}
