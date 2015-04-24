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

#include "PoseInfoContainer.h"

void PoseInfoContainer::update(const Eigen::Vector3f& slamPose, const Eigen::Matrix3f& slamCov, const ros::Time& stamp, const std::string& frame_id)
{
  //Fill stampedPose
  std_msgs::Header& header = stampedPose_.header;
  header.stamp = stamp;
  header.frame_id = frame_id;

  geometry_msgs::Pose& pose = stampedPose_.pose;
  pose.position.x = slamPose.x();
  pose.position.y = slamPose.y();

  pose.orientation.w = cos(slamPose.z()*0.5f);
  pose.orientation.z = sin(slamPose.z()*0.5f);

  //Fill covPose
  //geometry_msgs::PoseWithCovarianceStamped covPose;
  covPose_.header = header;
  covPose_.pose.pose = pose;

  boost::array<double, 36>& cov(covPose_.pose.covariance);

  cov[0] = static_cast<double>(slamCov(0,0));
  cov[7] = static_cast<double>(slamCov(1,1));
  cov[35] = static_cast<double>(slamCov(2,2));

  double xyC = static_cast<double>(slamCov(0,1));
  cov[1] = xyC;
  cov[6] = xyC;

  double xaC = static_cast<double>(slamCov(0,2));
  cov[5] = xaC;
  cov[30] = xaC;

  double yaC = static_cast<double>(slamCov(1,2));
  cov[11] = yaC;
  cov[31] = yaC;

  //Fill tf tansform
  tf::poseMsgToTF(pose, poseTransform_);
}
