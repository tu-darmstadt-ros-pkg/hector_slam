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

#ifndef __GridMapSimpleCount_h_
#define __GridMapSimpleCount_h_


/**
 * Provides a (very) simple count based representation of occupancy
 */
class SimpleCountCell
{
public:

  /**
   * Sets the cell value to val.
   * @param val The log odds value.
   */
  void set(float val)
  {
    simpleOccVal = val;
  }

  /**
   * Returns the value of the cell.
   * @return The log odds value.
   */
  float getValue() const
  {
    return simpleOccVal;
  }

  /**
   * Returns wether the cell is occupied.
   * @return Cell is occupied
   */
  bool isOccupied() const
  {
    return (simpleOccVal > 0.5f);
  }

  bool isFree() const
  {
    return (simpleOccVal < 0.5f);
  }

  /**
   * Reset Cell to prior probability.
   */
  void resetGridCell()
  {
    simpleOccVal = 0.5f;
    updateIndex = -1;
  }

//protected:

public:

  float simpleOccVal; ///< The log odds representation of occupancy probability.
  int updateIndex;


};

/**
 * Provides functions related to a log odds of occupancy probability respresentation for cells in a occupancy grid map.
 */
class GridMapSimpleCountFunctions
{
public:

  /**
   * Constructor, sets parameters like free and occupied log odds ratios.
   */
  GridMapSimpleCountFunctions()
  {
    updateFreeVal = -0.10f;
    updateOccVal  =  0.15f;

    updateFreeLimit = -updateFreeVal + updateFreeVal/100.0f;
    updateOccLimit  = 1.0f - (updateOccVal + updateOccVal/100.0f);
  }

  /**
   * Update cell as occupied
   * @param cell The cell.
   */
  void updateSetOccupied(SimpleCountCell& cell) const
  {
    if (cell.simpleOccVal < updateOccLimit){
      cell.simpleOccVal += updateOccVal;
    }
  }

  /**
   * Update cell as free
   * @param cell The cell.
   */
  void updateSetFree(SimpleCountCell& cell) const
  {
    if (cell.simpleOccVal > updateFreeLimit){
      cell.simpleOccVal += updateFreeVal;
    }
  }

  void updateUnsetFree(SimpleCountCell& cell) const
  {
    cell.simpleOccVal -= updateFreeVal;
  }

  /**
   * Get the probability value represented by the grid cell.
   * @param cell The cell.
   * @return The probability
   */
  float getGridProbability(const SimpleCountCell& cell) const
  {
    return cell.simpleOccVal;
  }

protected:

  float updateFreeVal;
  float updateOccVal;

  float updateFreeLimit;
  float updateOccLimit;


};


#endif
