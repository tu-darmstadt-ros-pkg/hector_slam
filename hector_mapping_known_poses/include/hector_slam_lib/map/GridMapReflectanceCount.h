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

#ifndef __GridMapReflectanceCount_h_
#define __GridMapReflectanceCount_h_

/**
 * Provides a reflectance count representation for cells in a occupancy grid map.
 */
class ReflectanceCell
{
public:

  void set(float val)
  {
    probOccupied = val;
  }

  float getValue() const
  {
    return probOccupied;
  }

  bool isOccupied() const
  {
    return probOccupied > 0.5f;
  }

  bool isFree() const{
    return probOccupied < 0.5f;
  }

  void resetGridCell()
  {
    probOccupied = 0.5f;
    visitedCount = 0.0f;
    reflectedCount = 0.0f;
    updateIndex = -1;
  }

//protected:

  float visitedCount;
  float reflectedCount;
  float probOccupied;
  int updateIndex;
};


class GridMapReflectanceFunctions
{
public:

  GridMapReflectanceFunctions()
  {}

  void updateSetOccupied(ReflectanceCell& cell) const
  {
    ++cell.reflectedCount;
    ++cell.visitedCount;
    cell.probOccupied = cell.reflectedCount / cell.visitedCount;
  }

  void updateSetFree(ReflectanceCell& cell) const
  {
    ++cell.visitedCount;
    cell.probOccupied = cell.reflectedCount / cell.visitedCount;
  }

  void updateUnsetFree(ReflectanceCell& cell) const
  {
    --cell.visitedCount;
    cell.probOccupied = cell.reflectedCount / cell.visitedCount;
  }

  float getGridProbability(const ReflectanceCell& cell) const
  {
    return cell.probOccupied;
  }

protected:

};


#endif
