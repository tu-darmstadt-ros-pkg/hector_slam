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

#ifndef _GEOTIFFWRITER_H__
#define _GEOTIFFWRITER_H__

#include "map_writer_interface.h"

#include <Eigen/Geometry>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <QtGui/QImage>
#include <QtGui/QApplication>
#include <QtGui/QFont>
#include <QtGui/QPen>

#include <hector_map_tools/HectorMapTools.h>


namespace hector_geotiff{


class GeotiffWriter : public MapWriterInterface
{
  public:
  GeotiffWriter(bool useCheckerboardCacheIn = false);
  virtual ~GeotiffWriter();

  //setUsePrecalcGrid(bool usePrecalc, const Eigen::Vector2f& size);

  void setMapFileName(const std::string& mapFileName);
  void setMapFilePath(const std::string& mapFilePath);
  void setUseUtcTimeSuffix(bool useSuffix);

  void setupImageSize();
  bool setupTransforms(const nav_msgs::OccupancyGrid& map);
  void drawBackgroundCheckerboard();
  void drawMap(const nav_msgs::OccupancyGrid& map, bool draw_explored_space_grid = true);
  void drawObjectOfInterest(const Eigen::Vector2f& coords, const std::string& txt, const Color& color);
  void drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points);
  void drawCoords();
  std::string getBasePathAndFileName() const;
  void writeGeotiffImage();


protected:

  void transformPainterToImgCoords(QPainter& painter);
  void drawCross(QPainter& painter, const Eigen::Vector2f& coords);
  void drawArrow(QPainter& painter);
  void drawCoordSystem(QPainter& painter);

  float resolution;
  Eigen::Vector2f origin;

  int resolutionFactor;
  float resolutionFactorf;

  bool useCheckerboardCache;
  bool use_utc_time_suffix_;

  float pixelsPerMapMeter;
  float pixelsPerGeoTiffMeter;

  Eigen::Vector2i minCoordsMap;
  Eigen::Vector2i maxCoordsMap;

  Eigen::Vector2i sizeMap;
  Eigen::Vector2f sizeMapf;

  Eigen::Vector2f rightBottomMarginMeters;
  Eigen::Vector2f rightBottomMarginPixelsf;
  Eigen::Vector2i rightBottomMarginPixels;

  Eigen::Vector2f leftTopMarginMeters;

  Eigen::Vector2f totalMeters;

  Eigen::Vector2i geoTiffSizePixels;

  Eigen::Vector2f mapOrigInGeotiff;
  Eigen::Vector2f mapEndInGeotiff;

  std::string map_file_name_;
  std::string map_file_path_;

  QImage image;
  QImage checkerboard_cache;
  QApplication* app;
  QFont map_draw_font_;

  HectorMapTools::CoordinateTransformer<float> world_map_transformer_;
  HectorMapTools::CoordinateTransformer<float> map_geo_transformer_;
  HectorMapTools::CoordinateTransformer<float> world_geo_transformer_;

  nav_msgs::MapMetaData cached_map_meta_data_;

  int fake_argc_;
  char** fake_argv_;
};

}

#endif
