//=================================================================================================
// Copyright (c) 2012, Gregor Gebhardt, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
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

#include <hector_geotiff/map_writer_interface.h>
#include <hector_geotiff/map_writer_plugin_interface.h>

#include <ros/ros.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>

#include <pluginlib/class_loader.h>
#include <fstream>

namespace hector_geotiff_plugins
{

using namespace hector_geotiff;

class TrajectoryMapWriter : public MapWriterPluginInterface
{
public:
  TrajectoryMapWriter();
  virtual ~TrajectoryMapWriter();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface *interface);

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient service_client_;

  bool initialized_;
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;
  int path_color_r_;
  int path_color_g_;
  int path_color_b_;
};

TrajectoryMapWriter::TrajectoryMapWriter()
    : initialized_(false)
{}

TrajectoryMapWriter::~TrajectoryMapWriter()
{}

void TrajectoryMapWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string service_name_;


  plugin_nh.param("service_name", service_name_, std::string("trajectory"));
  plugin_nh.param("path_color_r", path_color_r_, 120);
  plugin_nh.param("path_color_g", path_color_g_, 0);
  plugin_nh.param("path_color_b", path_color_b_, 240);

  service_client_ = nh_.serviceClient<hector_nav_msgs::GetRobotTrajectory>(service_name_);

  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized hector_geotiff MapWriter plugin %s.", name_.c_str());
}

void TrajectoryMapWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;

    hector_nav_msgs::GetRobotTrajectory srv_path;
    if (!service_client_.call(srv_path)) {
      ROS_ERROR_NAMED(name_, "Cannot draw trajectory, service %s failed", service_client_.getService().c_str());
      return;
    }

    std::vector<geometry_msgs::PoseStamped>& traj_vector (srv_path.response.trajectory.poses);

    size_t size = traj_vector.size();

    std::vector<Eigen::Vector2f> pointVec;
    pointVec.resize(size);

    for (size_t i = 0; i < size; ++i){
      const geometry_msgs::PoseStamped& pose (traj_vector[i]);

      pointVec[i] = Eigen::Vector2f(pose.pose.position.x, pose.pose.position.y);
    }

    if (size > 0){
      //Eigen::Vector3f startVec(pose_vector[0].x,pose_vector[0].y,pose_vector[0].z);
      Eigen::Vector3f startVec(pointVec[0].x(),pointVec[0].y(),0.0f);
      interface->drawPath(startVec, pointVec, path_color_r_, path_color_g_, path_color_b_);
    }
}

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(hector_geotiff_plugins::TrajectoryMapWriter, hector_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(hector_geotiff_plugins, TrajectoryMapWriter, hector_geotiff_plugins::TrajectoryMapWriter, hector_geotiff::MapWriterPluginInterface)
#endif
