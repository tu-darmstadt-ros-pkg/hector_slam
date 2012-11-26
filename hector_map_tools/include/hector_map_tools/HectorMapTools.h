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

#ifndef __HectorMapTools_h_
#define __HectorMapTools_h_

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include<Eigen/Core>

class HectorMapTools{
public:

  template<typename ConcreteScalar>
  class CoordinateTransformer{
  public:

    CoordinateTransformer()
    {

    }

    CoordinateTransformer(const nav_msgs::OccupancyGridConstPtr map)
    {
      this->setTransforms(*map);
    }


    void setTransforms(const nav_msgs::OccupancyGrid& map)
    {
      this->setTransforms(map.info);
    }

    void setTransforms(const nav_msgs::MapMetaData& meta)
    {
      origo_ = (Eigen::Matrix<ConcreteScalar, 2, 1>(static_cast<ConcreteScalar>(meta.origin.position.x),static_cast<ConcreteScalar>(meta.origin.position.y)));
      scale_ = (static_cast<ConcreteScalar>(meta.resolution));
      inv_scale_ = (static_cast<ConcreteScalar>(1.0f / meta.resolution));
    }

    void setTransformsBetweenCoordSystems(const Eigen::Matrix<ConcreteScalar, 2, 1>& origoCS1, const Eigen::Matrix<ConcreteScalar, 2, 1>& endCS1,
                                          const Eigen::Matrix<ConcreteScalar, 2, 1>& origoCS2, const Eigen::Matrix<ConcreteScalar, 2, 1>& endCS2)
    {
      Eigen::Matrix<ConcreteScalar, 2, 1> map_t_geotiff_x_factors = getLinearTransform(Eigen::Vector2f(origoCS1[0], endCS1[0]), Eigen::Vector2f(origoCS2[0], endCS2[0]));
      Eigen::Matrix<ConcreteScalar, 2, 1> map_t_geotiff_y_factors = getLinearTransform(Eigen::Vector2f(origoCS1[1], endCS1[1]), Eigen::Vector2f(origoCS2[1], endCS2[1]));

      //std::cout << "\n geo_x: \n" << map_t_geotiff_x_factors << "\n geo_y: \n" << map_t_geotiff_y_factors << "\n";

      origo_.x() = map_t_geotiff_x_factors[1];
      origo_.y() = map_t_geotiff_y_factors[1];

      scale_ = map_t_geotiff_x_factors[0];
      inv_scale_ = 1.0 / scale_;

      //std::cout << "\nscale " << scale_ << "\n";
    }

    Eigen::Matrix<ConcreteScalar, 2, 1> getC1Coords(const  Eigen::Matrix<ConcreteScalar, 2, 1>& mapCoords) const
    {
      return origo_ + (mapCoords * scale_);
    }

    Eigen::Matrix<ConcreteScalar, 2, 1> getC2Coords(const  Eigen::Matrix<ConcreteScalar, 2, 1>& worldCoords) const
    {
      return ((worldCoords - origo_) * inv_scale_);
    }

    ConcreteScalar getC1Scale(float c2_scale) const
    {
      return scale_ * c2_scale;
    }

    ConcreteScalar getC2Scale(float c1_scale) const
    {
      return inv_scale_ * c1_scale;
    }

  protected:

    Eigen::Matrix<ConcreteScalar, 2, 1> getLinearTransform(const Eigen::Matrix<ConcreteScalar, 2, 1>& coordSystem1, const Eigen::Matrix<ConcreteScalar, 2, 1>& coordSystem2)
    {
      ConcreteScalar scaling = (coordSystem2[0] - coordSystem2[1]) / (coordSystem1[0] - coordSystem1[1]);
      ConcreteScalar translation = coordSystem2[0] - coordSystem1[0] * scaling;
      return Eigen::Vector2f (scaling, translation);
    }

    Eigen::Matrix<ConcreteScalar, 2, 1> origo_;
    ConcreteScalar scale_;
    ConcreteScalar inv_scale_;
  };

  class DistanceMeasurementProvider
  {
  public:
    DistanceMeasurementProvider()
    {

    }

    void setMap(const nav_msgs::OccupancyGridConstPtr map)
    {
      map_ptr_ = map;

      world_map_transformer_.setTransforms(*map_ptr_);
    }

    float getDist(const Eigen::Vector2f& begin_world, const Eigen::Vector2f& end_world, Eigen::Vector2f* hitCoords = 0)
    {
      Eigen::Vector2i end_point_map;

      Eigen::Vector2i begin_map (world_map_transformer_.getC2Coords(begin_world).cast<int>());
      Eigen::Vector2i end_map (world_map_transformer_.getC2Coords(end_world).cast<int>());
      float dist = checkOccupancyBresenhami(begin_map, end_map, &end_point_map);

      if (hitCoords != 0){
        *hitCoords = world_map_transformer_.getC1Coords(end_point_map.cast<float>());
      }

      return world_map_transformer_.getC1Scale(dist);
    }

    inline float checkOccupancyBresenhami( const Eigen::Vector2i& beginMap, const Eigen::Vector2i& endMap, Eigen::Vector2i* hitCoords = 0, unsigned int max_length = UINT_MAX){

      int x0 = beginMap[0];
      int y0 = beginMap[1];

      int sizeX = map_ptr_->info.width;
      int sizeY = map_ptr_->info.height;

      //check if beam start point is inside map, cancel update if this is not the case
      if ((x0 < 0) || (x0 >= sizeX) || (y0 < 0) || (y0 >= sizeY)) {
        return -1.0f;
      }

      int x1 = endMap[0];
      int y1 = endMap[1];

      //std::cout << " x: "<< x1 << " y: " << y1 << " length: " << length << "     ";

      //check if beam end point is inside map, cancel update if this is not the case
      if ((x1 < 0) || (x1 >= sizeX) || (y1 < 0) || (y1 >= sizeY)) {
        return -1.0f;
      }

      int dx = x1 - x0;
      int dy = y1 - y0;

      unsigned int abs_dx = abs(dx);
      unsigned int abs_dy = abs(dy);

      int offset_dx =  dx > 0 ? 1 : -1;
      int offset_dy = (dy > 0 ? 1 : -1) * sizeX;

      unsigned int startOffset = beginMap.y() * sizeX + beginMap.x();

      int end_offset;

      //if x is dominant
      if(abs_dx >= abs_dy){
        int error_y = abs_dx / 2;
        end_offset = bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, startOffset,5000);
      }else{
        //otherwise y is dominant
        int error_x = abs_dy / 2;
        end_offset = bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, startOffset,5000);
      }

      if (end_offset != -1){
        Eigen::Vector2i end_coords(end_offset % sizeY, end_offset / sizeY);

        int distMap = ((beginMap - end_coords).cast<float>()).norm();

        if (hitCoords != 0){
          *hitCoords = end_coords;
        }

        return distMap;
      }

      return -1.0f;

      //unsigned int endOffset = endMap.y() * sizeX + endMap.x();
      //this->bresenhamCellOcc(endOffset);
    }

    inline int bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b,
                            unsigned int offset, unsigned int max_length){
      unsigned int end = std::min(max_length, abs_da);

      const std::vector<int8_t>& data = map_ptr_->data;

      for(unsigned int i = 0; i < end; ++i){
        if (data[offset] == 100){
          return static_cast<int>(offset);
        }

        offset += offset_a;
        error_b += abs_db;
        if((unsigned int)error_b >= abs_da){
          offset += offset_b;
          error_b -= abs_da;
        }
      }
      return -1;
      //at(offset);
    }

  protected:
    CoordinateTransformer<float> world_map_transformer_;
    nav_msgs::OccupancyGridConstPtr map_ptr_;


  };

  static bool getMapExtends(const nav_msgs::OccupancyGrid& map, Eigen::Vector2i& topLeft, Eigen::Vector2i& bottomRight)
  {
    int lowerStart = -1;
    int upperStart = 10000000;

    int xMaxTemp = lowerStart;
    int yMaxTemp = lowerStart;
    int xMinTemp = upperStart;
    int yMinTemp = upperStart;

    int width = map.info.width;
    int height = map.info.height;

    for (int y = 0; y < height; ++y){
      for (int x = 0; x < width; ++x){

        if ( map.data[x+y*width] != -1){

          if (x > xMaxTemp) {
            xMaxTemp = x;
          }

          if (x < xMinTemp) {
            xMinTemp = x;
          }

          if (y > yMaxTemp) {
            yMaxTemp = y;
          }

          if (y < yMinTemp) {
            yMinTemp = y;
          }
        }
      }
    }

    if ((xMaxTemp != lowerStart) &&
        (yMaxTemp != lowerStart) &&
        (xMinTemp != upperStart) &&
        (yMinTemp != upperStart)) {

      topLeft = Eigen::Vector2i(xMinTemp,yMinTemp);
      bottomRight = Eigen::Vector2i(xMaxTemp+1, yMaxTemp+1);

      return true;
    } else {
      return false;
    }
  };
};

#endif
