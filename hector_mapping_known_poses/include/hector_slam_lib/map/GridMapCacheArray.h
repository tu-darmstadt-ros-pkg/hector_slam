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

#ifndef __GridMapCacheArray_h_
#define __GridMapCacheArray_h_

#include <Eigen/Core>

class CachedMapElement
{
public:
  float val;
  int index;
};

/**
 * Caches filtered grid map accesses in a two dimensional array of the same size as the map.
 */
class GridMapCacheArray
{
public:

  /**
   * Constructor
   */
  GridMapCacheArray()
    : cacheArray(0)
    , arrayDimensions(-1,-1)
  {
    currCacheIndex = 0;
  }

  /**
   * Destructor
   */
  ~GridMapCacheArray()
  {
    deleteCacheArray();
  }

  /**
   * Resets/deletes the cached data
   */
  void resetCache()
  {
    currCacheIndex++;
  }

  /**
   * Checks wether cached data for coords is available. If this is the case, writes data into val.
   * @param coords The coordinates
   * @param val Reference to a float the data is written to if available
   * @return Indicates if cached data is available
   */
  bool containsCachedData(int index, float& val)
  {
    const CachedMapElement& elem (cacheArray[index]);

    if (elem.index == currCacheIndex) {
      val = elem.val;
      return true;
    } else {
      return false;
    }
  }

  /**
   * Caches float value val for coordinates coords.
   * @param coords The coordinates
   * @param val The value to be cached for coordinates.
   */
  void cacheData(int index, float val)
  {
    CachedMapElement& elem (cacheArray[index]);
    elem.index = currCacheIndex;
    elem.val = val;
  }

  /**
   * Sets the map size and resizes the cache array accordingly
   * @param sizeIn The map size.
   */
  void setMapSize(const Eigen::Vector2i& newDimensions)
  {
    setArraySize(newDimensions);
  }

protected:

  /**
   * Creates a cache array of size sizeIn.
   * @param sizeIn The size of the array
   */
  void createCacheArray(const Eigen::Vector2i& newDimensions)
  {
    arrayDimensions = newDimensions;

    int sizeX = arrayDimensions[0];
    int sizeY = arrayDimensions[1];

    int size = sizeX * sizeY;

    cacheArray = new CachedMapElement [size];

    for (int x = 0; x < size; ++x) {
      cacheArray[x].index = -1;
    }
  }

  /**
   * Deletes the existing cache array.
   */
  void deleteCacheArray()
  {
    delete[] cacheArray;
  }

  /**
   * Sets a new cache array size
   */
  void setArraySize(const Eigen::Vector2i& newDimensions)
  {
    if (this->arrayDimensions != newDimensions) {
      if (cacheArray != 0) {
        deleteCacheArray();
        cacheArray = 0;
      }
      createCacheArray(newDimensions);
    }
  }

protected:

  CachedMapElement* cacheArray;    ///< Array used for caching data.
  int currCacheIndex;              ///< The cache iteration index value

  Eigen::Vector2i arrayDimensions; ///< The size of the array

};


#endif
