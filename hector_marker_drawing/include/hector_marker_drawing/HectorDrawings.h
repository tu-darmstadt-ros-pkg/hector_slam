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

#include "DrawInterface.h"
//#include "util/UtilFunctions.h"

#include "ros/ros.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>


#include <visualization_msgs/MarkerArray.h>

class HectorDrawings : public DrawInterface
{
public:

  HectorDrawings()
  {
    idCounter = 0;
    maxId = 0;

    ros::NodeHandle nh_;

    markerPublisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    markerArrayPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);

    tempMarker.header.frame_id = "map";
    tempMarker.ns = "marker";

    this->setScale(1.0);
    this->setColor(1.0, 1.0, 1.0);

    tempMarker.action = visualization_msgs::Marker::ADD;
  };

  virtual void setNamespace(const std::string& ns)
  {
    tempMarker.ns = ns;
  }

  virtual void drawPoint(const Eigen::Vector2f& pointWorldFrame)
  {
    tempMarker.id = idCounter++;

    tempMarker.pose.position.x = pointWorldFrame.x();
    tempMarker.pose.position.y = pointWorldFrame.y();

    tempMarker.pose.orientation.w = 0.0;
    tempMarker.pose.orientation.z = 0.0;
    tempMarker.type = visualization_msgs::Marker::CUBE;

    //markerPublisher_.publish(tempMarker);

    markerArray.markers.push_back(tempMarker);
  }

  virtual void drawArrow(const Eigen::Vector3f& poseWorld)
  {
    tempMarker.id = idCounter++;

    tempMarker.pose.position.x = poseWorld.x();
    tempMarker.pose.position.y = poseWorld.y();

    tempMarker.pose.orientation.w = cos(poseWorld.z()*0.5f);
    tempMarker.pose.orientation.z = sin(poseWorld.z()*0.5f);

    tempMarker.type = visualization_msgs::Marker::ARROW;

    //markerPublisher_.publish(tempMarker);

    markerArray.markers.push_back(tempMarker);

  }

  virtual void drawCovariance(const Eigen::Vector2f& mean, const Eigen::Matrix2f& covMatrix)
  {

    tempMarker.pose.position.x = mean[0];
    tempMarker.pose.position.y = mean[1];

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covMatrix);

    const Eigen::Vector2f& eigValues (eig.eigenvalues());
    const Eigen::Matrix2f& eigVectors (eig.eigenvectors());

    float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));

    tempMarker.type = visualization_msgs::Marker::CYLINDER;

    double lengthMajor = sqrt(eigValues[0]);
    double lengthMinor = sqrt(eigValues[1]);

    tempMarker.scale.x = lengthMajor;
    tempMarker.scale.y = lengthMinor;
    tempMarker.scale.z = 0.001;


    tempMarker.pose.orientation.w = cos(angle*0.5);
    tempMarker.pose.orientation.z = sin(angle*0.5);

    tempMarker.id = idCounter++;
    markerArray.markers.push_back(tempMarker);
  }

  virtual void drawCovariance(const Eigen::Vector3f& mean, const Eigen::Matrix3f& covMatrix)
  {
    tempMarker.type = visualization_msgs::Marker::SPHERE;

    tempMarker.color.r = 0.0;
    tempMarker.color.a = 0.5;

    tempMarker.pose.position.x = mean[0];
    tempMarker.pose.position.y = mean[1];
    tempMarker.pose.position.z = mean[2];

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covMatrix);

    const Eigen::Vector3f& eigValues (eig.eigenvalues());
    const Eigen::Matrix3f& eigVectors (eig.eigenvectors());


    Eigen::Matrix3f eigVectorsFlipped;
    eigVectorsFlipped.col(0) = eigVectors.col(2);
    eigVectorsFlipped.col(1) = eigVectors.col(1);
    eigVectorsFlipped.col(2) = eigVectors.col(0);

    if (eigVectorsFlipped.determinant() < 0){
      eigVectorsFlipped.col(2) = -eigVectorsFlipped.col(2);
    }

    Eigen::Quaternionf quaternion (eigVectorsFlipped);

    //std::cout << "\neigVec:\n" << eigVectors << "\n";
    //std::cout << "\neigVecFlipped:\n" << eigVectorsFlipped << "\n";

    //std::cout << "\now:" << quaternion.w() << " x:" << quaternion.x() << " y:" << quaternion.y() << " z:" << quaternion.z() << "\n";


    tempMarker.pose.orientation.w = quaternion.w();
    tempMarker.pose.orientation.x = quaternion.x();
    tempMarker.pose.orientation.y = quaternion.y();
    tempMarker.pose.orientation.z = quaternion.z();

    tempMarker.scale.x = sqrt(eigValues[2]);
    tempMarker.scale.y = sqrt(eigValues[1]);
    tempMarker.scale.z = sqrt(eigValues[0]);

    tempMarker.id = idCounter++;
    markerArray.markers.push_back(tempMarker);


  }

  virtual void setScale(double scale)
  {
    tempMarker.scale.x = scale;
    tempMarker.scale.y = scale;
    tempMarker.scale.z = scale;
  }

  virtual void setColor(double r, double g, double b, double a = 1.0)
  {
    tempMarker.color.r = r;
    tempMarker.color.g = g;
    tempMarker.color.b = b;
    tempMarker.color.a = a;
  }

  virtual void addMarker(visualization_msgs::Marker marker) {
    if (marker.id == 0) marker.id = idCounter++;
    if (marker.ns.empty()) marker.ns = tempMarker.ns;
    markerArray.markers.push_back(marker);
  }

  virtual void addMarkers(visualization_msgs::MarkerArray markers) {
    for(visualization_msgs::MarkerArray::_markers_type::iterator it = markers.markers.begin(); it != markers.markers.end(); ++it) {
      visualization_msgs::Marker &marker = *it;
      addMarker(marker);
    }
  }

  virtual void sendAndResetData()
  {
    allMarkers.markers.insert(allMarkers.markers.end(), markerArray.markers.begin(), markerArray.markers.end());
    markerArrayPublisher_.publish(markerArray);
    markerArray.markers.clear();
    if (idCounter > maxId) maxId = idCounter;
    idCounter = 0;
  }

  void setTime(const ros::Time& time)
  {
    tempMarker.header.stamp = time;
  }

  void reset()
  {
    for(visualization_msgs::MarkerArray::_markers_type::iterator it = allMarkers.markers.begin(); it != allMarkers.markers.end(); ++it)
    {
      it->action = visualization_msgs::Marker::DELETE;
    }
    markerArrayPublisher_.publish(allMarkers);
    allMarkers.markers.clear();
  }

  ros::Publisher markerPublisher_;
  ros::Publisher markerArrayPublisher_;

  visualization_msgs::Marker tempMarker;
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::MarkerArray allMarkers;

  int idCounter;
  int maxId;
};
