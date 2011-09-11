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

#ifndef _hectormapproccontainer_h__
#define _hectormapproccontainer_h__

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"
#include "../util/MapLockerInterface.h"

class GridMap;
class ConcreteOccGridMapUtil;
class DataContainer;

namespace hectorslam{

class MapProcContainer
{
public:
  MapProcContainer(GridMap* gridMapIn, OccGridMapUtilConfig<GridMap>* gridMapUtilIn, ScanMatcher<OccGridMapUtilConfig<GridMap> >* scanMatcherIn)
    : gridMap(gridMapIn)
    , gridMapUtil(gridMapUtilIn)
    , scanMatcher(scanMatcherIn)
    , mapMutex(0)
  {}

  virtual ~MapProcContainer()
  {}

  void cleanup()
  {
    delete gridMap;
    delete gridMapUtil;
    delete scanMatcher;

    if (mapMutex){
      delete mapMutex;
    }
  }

  void reset()
  {
    gridMap->reset();
    gridMapUtil->resetCachedData();
  }

  void resetCachedData()
  {
    gridMapUtil->resetCachedData();
  }

  float getScaleToMap() const { return gridMap->getScaleToMap(); };

  const GridMap& getGridMap() const { return *gridMap; };
  GridMap& getGridMap() { return *gridMap; };

  void addMapMutex(MapLockerInterface* mapMutexIn)
  {
    if (mapMutex)
    {
      delete mapMutex;
    }

    mapMutex = mapMutexIn;
  }

  MapLockerInterface* getMapMutex()
  {
    return mapMutex;
  }

  Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix, int maxIterations)
  {
    return scanMatcher->matchData(beginEstimateWorld, *gridMapUtil, dataContainer, covMatrix, maxIterations);
  }

  void updateByScan(const DataContainer& dataContainer, const Eigen::Vector3f& robotPoseWorld)
  {
    if (mapMutex)
    {
      mapMutex->lockMap();
    }

    gridMap->updateByScan(dataContainer, robotPoseWorld);

    if (mapMutex)
    {
      mapMutex->unlockMap();
    }
  }

  GridMap* gridMap;
  OccGridMapUtilConfig<GridMap>* gridMapUtil;
  ScanMatcher<OccGridMapUtilConfig<GridMap> >* scanMatcher;
  MapLockerInterface* mapMutex;
};

}

#endif
