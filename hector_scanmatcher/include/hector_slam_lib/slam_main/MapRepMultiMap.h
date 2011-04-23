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

#ifndef _hectormaprepmultimap_h__
#define _hectormaprepmultimap_h__

#include "MapRepresentationInterface.h"
#include "MapProcContainer.h"

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"

#include "../util/DrawInterface.h"
#include "../util/HectorDebugInfoInterface.h"

namespace hectorslam{

class MapRepMultiMap : public MapRepresentationInterface
{

public:
  MapRepMultiMap(float mapResolution, int mapSize, unsigned int numDepth, const Eigen::Vector2f& startCoords, DrawInterface* drawInterfaceIn, HectorDebugInfoInterface* debugInterfaceIn)
  {
    //unsigned int numDepth = 3;
    Eigen::Vector2i resolution(mapSize, mapSize);

    float totalMapSize = mapResolution * static_cast<float>(mapSize);
    float midOffset = totalMapSize * 0.5f;

    for (unsigned int i = 0; i < numDepth; ++i){
      std::cout << "cellLength: " << mapResolution << " res:\n " << resolution << "\n";
      GridMap* gridMap = new hectorslam::GridMap(mapResolution,resolution, Eigen::Vector2f(midOffset, midOffset));
      OccGridMapUtilConfig<GridMap>* gridMapUtil = new OccGridMapUtilConfig<GridMap>(gridMap);
      ScanMatcher<OccGridMapUtilConfig<GridMap> >* scanMatcher = new hectorslam::ScanMatcher<OccGridMapUtilConfig<GridMap> >(drawInterfaceIn, debugInterfaceIn);

      mapContainer.push_back(MapProcContainer(gridMap, gridMapUtil, scanMatcher));

      resolution /= 2;
      mapResolution*=2.0f;
    }

    dataContainers.resize(numDepth-1);

  }

  virtual ~MapRepMultiMap()
  {
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      mapContainer[i].cleanup();
    }
  }

  virtual void reset()
  {
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      mapContainer[i].reset();
    }
  }

  virtual float getScaleToMap() const { return mapContainer[0].getScaleToMap(); };

  virtual int getMapLevels() const { return mapContainer.size(); };
  virtual const GridMap& getGridMap(int mapLevel) const { return mapContainer[mapLevel].getGridMap(); };

  virtual void addMapMutex(int i, MapLockerInterface* mapMutex)
  {
    mapContainer[i].addMapMutex(mapMutex);
  }

  MapLockerInterface* getMapMutex(int i)
  {
    return mapContainer[i].getMapMutex();
  }

  virtual void onMapUpdated()
  {
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      mapContainer[i].resetCachedData();
    }
  }

  virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix)
  {
    int size = mapContainer.size();

    Eigen::Vector3f tmp(beginEstimateWorld);

    for (int i = size-1; i >=0; --i){
      //std::cout << " m " << i;
      if (i == 0){
        tmp  = (mapContainer[i].matchData(tmp, dataContainer, covMatrix, 13));
      }else{
        dataContainers[i-1].setFrom(dataContainer, static_cast<float>(1.0 / pow(2.0, static_cast<double>(i))));
        tmp  = (mapContainer[i].matchData(tmp, dataContainers[i-1], covMatrix, 5));
      }
    }
    return tmp;
  }

  virtual void updateByScan(const DataContainer& dataContainer, const Eigen::Vector3f& robotPoseWorld)
  {
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      //std::cout << " u " <<  i;
      if (i==0){
        mapContainer[i].updateByScan(dataContainer, robotPoseWorld);
      }else{
        mapContainer[i].updateByScan(dataContainers[i-1], robotPoseWorld);
      }
    }
    //std::cout << "\n";
  }

protected:
  std::vector<MapProcContainer> mapContainer;
  std::vector<DataContainer> dataContainers;
};

}

#endif
