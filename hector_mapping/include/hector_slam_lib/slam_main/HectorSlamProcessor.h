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

#ifndef _hectorslamprocessor_h__
#define _hectorslamprocessor_h__

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"
#include "../scan/DataPointContainer.h"

#include "../util/UtilFunctions.h"
#include "../util/DrawInterface.h"
#include "../util/HectorDebugInfoInterface.h"
#include "../util/MapLockerInterface.h"

#include "MapRepresentationInterface.h"
#include "MapRepMultiMap.h"


#include <float.h>

namespace hectorslam{

class HectorSlamProcessor
{
public:

  HectorSlamProcessor(float mapResolution, int mapSizeX, int mapSizeY , const Eigen::Vector2f& startCoords, int multi_res_size, DrawInterface* drawInterfaceIn = 0, HectorDebugInfoInterface* debugInterfaceIn = 0)
    : drawInterface(drawInterfaceIn)
    , debugInterface(debugInterfaceIn)
  {
    mapRep = new MapRepMultiMap(mapResolution, mapSizeX, mapSizeY, multi_res_size, startCoords, drawInterfaceIn, debugInterfaceIn);

    this->reset();

    this->setMapUpdateMinDistDiff(0.4f *1.0f);
    this->setMapUpdateMinAngleDiff(0.13f * 1.0f);
  }

  ~HectorSlamProcessor()
  {
    delete mapRep;
  }

  void update(const DataContainer& dataContainer, const Eigen::Vector3f& poseHintWorld, bool map_without_matching = false)
  {
    //std::cout << "\nph:\n" << poseHintWorld << "\n";

    Eigen::Vector3f newPoseEstimateWorld;

    if (!map_without_matching){
        newPoseEstimateWorld = (mapRep->matchData(poseHintWorld, dataContainer, lastScanMatchCov));
    }else{
        newPoseEstimateWorld = poseHintWorld;
    }

    lastScanMatchPose = newPoseEstimateWorld;

    //std::cout << "\nt1:\n" << newPoseEstimateWorld << "\n";

    //std::cout << "\n1";
    //std::cout << "\n" << lastScanMatchPose << "\n";
    if(util::poseDifferenceLargerThan(newPoseEstimateWorld, lastMapUpdatePose, paramMinDistanceDiffForMapUpdate, paramMinAngleDiffForMapUpdate) || map_without_matching){

      mapRep->updateByScan(dataContainer, newPoseEstimateWorld);

      mapRep->onMapUpdated();
      lastMapUpdatePose = newPoseEstimateWorld;
    }

    if(drawInterface){
      const GridMap& gridMapRef (mapRep->getGridMap());
      drawInterface->setColor(1.0, 0.0, 0.0);
      drawInterface->setScale(0.15);

      drawInterface->drawPoint(gridMapRef.getWorldCoords(Eigen::Vector2f::Zero()));
      drawInterface->drawPoint(gridMapRef.getWorldCoords((gridMapRef.getMapDimensions().array()-1).cast<float>()));
      drawInterface->drawPoint(Eigen::Vector2f(1.0f, 1.0f));

      drawInterface->sendAndResetData();
    }

    if (debugInterface)
    {
      debugInterface->sendAndResetData();
    }
  }

  void reset()
  {
    lastMapUpdatePose = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
    lastScanMatchPose = Eigen::Vector3f::Zero();
    //lastScanMatchPose.x() = -10.0f;
    //lastScanMatchPose.y() = -15.0f;
    //lastScanMatchPose.z() = M_PI*0.15f;

    mapRep->reset();
  }

  const Eigen::Vector3f& getLastScanMatchPose() const { return lastScanMatchPose; };
  const Eigen::Matrix3f& getLastScanMatchCovariance() const { return lastScanMatchCov; };
  float getScaleToMap() const { return mapRep->getScaleToMap(); };

  int getMapLevels() const { return mapRep->getMapLevels(); };
  const GridMap& getGridMap(int mapLevel = 0) const { return mapRep->getGridMap(mapLevel); };

  void addMapMutex(int i, MapLockerInterface* mapMutex) { mapRep->addMapMutex(i, mapMutex); };
  MapLockerInterface* getMapMutex(int i) { return mapRep->getMapMutex(i); };

  void setUpdateFactorFree(float free_factor) { mapRep->setUpdateFactorFree(free_factor); };
  void setUpdateFactorOccupied(float occupied_factor) { mapRep->setUpdateFactorOccupied(occupied_factor); };
  void setMapUpdateMinDistDiff(float minDist) { paramMinDistanceDiffForMapUpdate = minDist; };
  void setMapUpdateMinAngleDiff(float angleChange) { paramMinAngleDiffForMapUpdate = angleChange; };

protected:

  MapRepresentationInterface* mapRep;

  Eigen::Vector3f lastMapUpdatePose;
  Eigen::Vector3f lastScanMatchPose;
  Eigen::Matrix3f lastScanMatchCov;

  float paramMinDistanceDiffForMapUpdate;
  float paramMinAngleDiffForMapUpdate;

  DrawInterface* drawInterface;
  HectorDebugInfoInterface* debugInterface;
};

}

#endif
