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

#ifndef utilfunctions_h__
#define utilfunctions_h__

#include <cmath>

namespace util{

static inline float normalize_angle_pos(float angle)
{
  return fmod(fmod(angle, 2.0f*M_PI) + 2.0f*M_PI, 2.0f*M_PI);
}

static inline float normalize_angle(float angle)
{
  float a = normalize_angle_pos(angle);
  if (a > M_PI){
    a -= 2.0f*M_PI;
  }
  return a;
}

static inline float sqr(float val)
{
  return val*val;
}

static inline int sign(int x)
{
  return x > 0 ? 1 : -1;
}

template<typename T>
static T toDeg(const T radVal)
{
  return radVal * static_cast<T>(180.0 / M_PI);
}

template<typename T>
static T toRad(const T degVal)
{
  return degVal * static_cast<T>(M_PI / 180.0);
}

static bool poseDifferenceLargerThan(const Eigen::Vector3f& pose1, const Eigen::Vector3f& pose2, float distanceDiffThresh, float angleDiffThresh)
{
  //check distance
  if ( ( (pose1.head<2>() - pose2.head<2>()).norm() ) > distanceDiffThresh){
    return true;
  }

  float angleDiff = (pose1.z() - pose2.z());

  if (angleDiff > M_PI) {
    angleDiff -= M_PI * 2.0f;
  } else if (angleDiff < -M_PI) {
    angleDiff += M_PI * 2.0f;
  }

  if (abs(angleDiff) > angleDiffThresh){
    return true;
  }
  return false;
}

}

#endif
