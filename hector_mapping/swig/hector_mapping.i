%module HectorSlam
%{
#define SWIG_FILE_WITH_INIT
%}
%{
#include <iostream>
#include <Eigen/Core>

Eigen::Vector2f * Eigen_Vector2f(float x, float y){
  return new Eigen::Vector2f(x, y);
}
float Eigen_Vector2f_x(Eigen::Vector2f * v){ return v->x(); }
float Eigen_Vector2f_y(Eigen::Vector2f * v){ return v->y(); }

float Eigen_Vector3f_x(Eigen::Vector3f * v){ return v->x(); }
float Eigen_Vector3f_y(Eigen::Vector3f * v){ return v->y(); }
float Eigen_Vector3f_z(Eigen::Vector3f * v){ return v->z(); }

#include "include/hector_slam_lib/slam_main/HectorSlamProcessor.h"
#include "include/hector_slam_lib/scan/DataPointContainer.h"
#include "include/hector_slam_lib/map/GridMap.h"
%}

Eigen::Vector2f * Eigen_Vector2f(float x, float y);
float Eigen_Vector2f_x(Eigen::Vector2f * v);
float Eigen_Vector2f_y(Eigen::Vector2f * v);

float Eigen_Vector3f_x(Eigen::Vector3f * v);
float Eigen_Vector3f_y(Eigen::Vector3f * v);
float Eigen_Vector3f_z(Eigen::Vector3f * v);


/*************************************************************/
%include <Eigen/Core>
#define  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // dummy defined to pass swig
// IGNORE : GridMapBase.h:349:30: error: no match for ‘operator[]’ (operand types are ‘LogOddsCell’ and ‘int’)
%ignore  hectorslam::GridMapBase::getMapExtends(int& xMax, int& yMax, int& xMin, int& yMin) const;
%include "include/hector_slam_lib/map/GridMapLogOdds.h"
%include "include/hector_slam_lib/map/GridMapBase.h"
%include "include/hector_slam_lib/map/OccGridMapBase.h"
%include "include/hector_slam_lib/scan/DataPointContainer.h"
%include "include/hector_slam_lib/slam_main/HectorSlamProcessor.h"

%template(DataPointContainer_Eigen_Vector2f) hectorslam::DataPointContainer<Eigen::Vector2f>;
namespace hectorslam {
  %template(GridMap) OccGridMapBase<LogOddsCell, GridMapLogOddsFunctions>;
  %template(GridMapBase_LogOddsCell) GridMapBase<LogOddsCell>;

}






