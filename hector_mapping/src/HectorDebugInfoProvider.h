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

#ifndef HECTOR_DEBUG_INFO_PROVIDER_H__
#define HECTOR_DEBUG_INFO_PROVIDER_H__

#include "util/HectorDebugInfoInterface.h"
#include "util/UtilFunctions.h"

#include "ros/ros.h"

#include "hector_mapping/HectorDebugInfo.h"


class HectorDebugInfoProvider : public HectorDebugInfoInterface
{
public:

  HectorDebugInfoProvider()
  {
    ros::NodeHandle nh_;

    debugInfoPublisher_ = nh_.advertise<hector_mapping::HectorDebugInfo>("hector_debug_info", 50, true);
  };

  virtual void sendAndResetData()
  {
    debugInfoPublisher_.publish(debugInfo);
    debugInfo.iterData.clear();
  }


  virtual void addHessianMatrix(const Eigen::Matrix3f& hessian)
  {
    hector_mapping::HectorIterData iterData;

    for (int i=0; i < 9; ++i){
      iterData.hessian[i] = static_cast<double>(hessian.data()[i]);
      iterData.determinant = hessian.determinant();

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(hessian);

      const Eigen::Vector3f& eigValues (eig.eigenvalues());
      iterData.conditionNum = eigValues[2] / eigValues[0];


      iterData.determinant2d = hessian.block<2,2>(0,0).determinant();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig2d(hessian.block<2,2>(0,0));

      const Eigen::Vector2f& eigValues2d (eig2d.eigenvalues());
      iterData.conditionNum2d = eigValues2d[1] / eigValues2d[0];
    }

    debugInfo.iterData.push_back(iterData);
  }

  virtual void addPoseLikelihood(float lh)
  {

  }


  hector_mapping::HectorDebugInfo debugInfo;

  ros::Publisher debugInfoPublisher_;

};

#endif
