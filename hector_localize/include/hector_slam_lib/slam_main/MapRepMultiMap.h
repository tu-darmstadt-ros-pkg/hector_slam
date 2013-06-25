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
// (INCLUDING, BUT NOT LIMITE-D TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef _hectormaprepmultimap_h__
#define _hectormaprepmultimap_h__

#include "ros/ros.h"

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
  MapRepMultiMap(float mapResolution, int mapSizeX, int mapSizeY, unsigned int numDepth, const Eigen::Vector2f& startCoords, bool update_given_map_, DrawInterface* drawInterfaceIn, HectorDebugInfoInterface* debugInterfaceIn)
  {

    ROS_DEBUG("Initializing the MapRepMultiMap");
    //unsigned int numDepth = 3;
    Eigen::Vector2i resolution(mapSizeX, mapSizeY);
  
    float totalMapSizeX = mapResolution * static_cast<float>(mapSizeX);
    float mid_offset_x = totalMapSizeX * startCoords.x();

    float totalMapSizeY = mapResolution * static_cast<float>(mapSizeY);
    float mid_offset_y = totalMapSizeY * startCoords.y();

    for (unsigned int i = 0; i < numDepth; ++i){
      std::cout << "HectorSM map lvl " << i << ": cellLength: " << mapResolution << " res x:" << resolution.x() << " res y: " << resolution.y() << "\n";
      GridMap* gridMap = new hectorslam::GridMap(mapResolution,resolution, Eigen::Vector2f(mid_offset_x, mid_offset_y), update_given_map_);
      OccGridMapUtilConfig<GridMap>* gridMapUtil = new OccGridMapUtilConfig<GridMap>(gridMap);
      ScanMatcher<OccGridMapUtilConfig<GridMap> >* scanMatcher = new hectorslam::ScanMatcher<OccGridMapUtilConfig<GridMap> >(drawInterfaceIn, debugInterfaceIn);
      if(i != numDepth-1){
      	mapContainer.push_back(MapProcContainer(gridMap, gridMapUtil, scanMatcher));
      }
      
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
      //ROS_WARN("HEY I get called");
      mapContainer[i].resetCachedData();
    }
  }

  virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix)
  {
    
    size_t size = mapContainer.size();

    Eigen::Vector3f tmp(beginEstimateWorld);
    //ROS_WARN("tmp index: %f %f %f", tmp(0), tmp(1), tmp(2));

    size_t index = size - 1;
    for (size_t i = 0; i < size; ++i){
      //std::cout << " m " << i;
      if (index == 0){
        tmp  = (mapContainer[index].matchData(tmp, dataContainer, covMatrix, 5));
	//ROS_WARN("tmp index = 0: %f %f %f", tmp(0), tmp(1), tmp(2));
      }else{
        dataContainers[index-1].setFrom(dataContainer, static_cast<float>(1.0 / pow(2.0, static_cast<double>(index))));
        tmp  = (mapContainer[index].matchData(tmp, dataContainers[index-1], covMatrix, 3));
	//	ROS_WARN("tmp index != 0: %f %f %f", tmp(0), tmp(1), tmp(2));
      }
    }
    //ROS_WARN(dataContainers[index]);
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

  virtual void setUpdateFactorFree(float free_factor)
  {
    size_t size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      GridMap& map = mapContainer[i].getGridMap();
      map.setUpdateFreeFactor(free_factor);
    }
  }

  virtual void setUpdateFactorOccupied(float occupied_factor)
  {
    size_t size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      GridMap& map = mapContainer[i].getGridMap();
      map.setUpdateOccupiedFactor(occupied_factor);
    }
  }

  /*
   * This function takes as input the data from an occupancyGrid message
   * and then uses it to recreate the occupancyGrid within the confines of
   * a GridMap
   */
  // virtual void staticGridUpdate(std::vector<signed char, std::allocator<signed char> > gridData, int mapLevel)
  virtual void staticGridUpdate(float* gridData, int mapLevel, const Eigen::Vector2f offset, float map_resolution)
  {
    ROS_DEBUG("Running static Grid update");

    //First pull the gridMap out of the map container
    GridMap& map = mapContainer[mapLevel].getGridMap();
    //Then update the grid data
    map.staticGridUpdate(gridData);
    //update the offset of the map
    map.setMapTransformation(offset, map_resolution);
  }

protected:
  std::vector<MapProcContainer> mapContainer;
  std::vector<DataContainer> dataContainers;
};

}

#endif
