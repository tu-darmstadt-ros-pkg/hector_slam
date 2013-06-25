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

#ifndef __MapDimensionProperties_h_
#define __MapDimensionProperties_h_

class MapDimensionProperties
{
public:
  MapDimensionProperties()
    : topLeftOffset(-1.0f,-1.0f)
    , mapDimensions(-1,-1)
    , cellLength(-1.0f)
  {}


  MapDimensionProperties(const Eigen::Vector2f& topLeftOffsetIn, const Eigen::Vector2i& mapDimensionsIn, float cellLengthIn)
    : topLeftOffset(topLeftOffsetIn)
    , mapDimensions(mapDimensionsIn)
    , cellLength(cellLengthIn)
  {
    mapLimitsf = (mapDimensionsIn.cast<float>()).array() - 1.0f;
  }

  bool operator==(const MapDimensionProperties& other) const
  {
    return (topLeftOffset == other.topLeftOffset) && (mapDimensions == other.mapDimensions) && (cellLength == other.cellLength);
  }

  bool hasEqualDimensionProperties(const MapDimensionProperties& other) const
  {
    return (mapDimensions == other.mapDimensions);
  }

  bool hasEqualTransformationProperties(const MapDimensionProperties& other) const
  {
     return  (topLeftOffset == other.topLeftOffset) && (cellLength == other.cellLength);
  }

  bool pointOutOfMapBounds(const Eigen::Vector2f& coords) const
  {
    return ((coords[0] < 0.0f) || (coords[0] > mapLimitsf[0]) || (coords[1] < 0.0f) || (coords[1] > mapLimitsf[1]));
  }

  void setMapCellDims(const Eigen::Vector2i& newDims)
  {
    mapDimensions = newDims;
    mapLimitsf = (newDims.cast<float>()).array() - 2.0f;
  }

  void setTopLeftOffset(const Eigen::Vector2f& topLeftOffsetIn)
  {
    topLeftOffset = topLeftOffsetIn;
  }

  void setSizeX(int sX) { mapDimensions[0] = sX; };
  void setSizeY(int sY) { mapDimensions[1] = sY; };
  void setCellLength(float cl) { cellLength = cl; };

  const Eigen::Vector2f& getTopLeftOffset() const { return topLeftOffset; };
  const Eigen::Vector2i& getMapDimensions() const { return mapDimensions; };
  int getSizeX() const { return mapDimensions[0]; };
  int getSizeY() const { return mapDimensions[1]; };
  float getCellLength() const { return cellLength; };

protected:
  Eigen::Vector2f topLeftOffset;
  Eigen::Vector2i mapDimensions;
  Eigen::Vector2f mapLimitsf;
  float cellLength;
};

#endif


