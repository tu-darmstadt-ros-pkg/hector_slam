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

#ifndef _MAPWRITERINTERFACE_H__
#define _MAPWRITERINTERFACE_H__

#include <vector>
#include <Eigen/Core>

namespace hector_geotiff{

class MapWriterInterface{
public:
  struct Color {
    Color(unsigned int r, unsigned int g, unsigned int b) : r(r), g(g), b(b) {}
    unsigned int r,g,b;
  };

  virtual std::string getBasePathAndFileName() const = 0;
  virtual void drawObjectOfInterest(const Eigen::Vector2f& coords, const std::string& txt, const Color& color) = 0;
  //virtual void drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points) = 0;

  inline virtual void drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points){
      drawPath(start, points, 120,0,240);
  }
  virtual void drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points, int color_r, int color_g, int color_b) = 0;

};

}

#endif
