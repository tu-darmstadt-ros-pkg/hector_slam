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
#include "sensor_msgs/Imu.h"

std::string p_base_stabilized_frame_;
std::string p_base_frame_;
tf::TransformBroadcaster* tfB_;
tf::StampedTransform transform_;
tf::Quaternion tmp_;

#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void imuMsgCallback(const sensor_msgs::Imu& imu_msg)
{
  tf::quaternionMsgToTF(imu_msg.orientation, tmp_);

  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);

  tmp_.setRPY(roll, pitch, 0.0);

  transform_.setRotation(tmp_);

  transform_.stamp_ = imu_msg.header.stamp;

  tfB_->sendTransform(transform_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pn.param("base_stabilized_frame", p_base_stabilized_frame_, std::string("base_stabilized"));
  pn.param("base_frame", p_base_frame_, std::string("base_link"));
  
  tfB_ = new tf::TransformBroadcaster();
  transform_.getOrigin().setX(0.0);
  transform_.getOrigin().setY(0.0);
  transform_.getOrigin().setZ(0.0);
  transform_.frame_id_ = p_base_stabilized_frame_;
  transform_.child_frame_id_ = p_base_frame_;

  ros::Subscriber imu_subscriber = n.subscribe("imu_topic", 10, imuMsgCallback);

  ros::spin();

  delete tfB_;

  return 0;
}
