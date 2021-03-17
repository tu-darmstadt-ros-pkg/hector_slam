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

#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <Eigen/Geometry>

#include <hector_map_tools/HectorMapTools.h>

using namespace std;

/**
 * @brief This node provides occupancy grid maps as images via image_transport, so the transmission consumes less bandwidth.
 * The provided code is a incomplete proof of concept.
 */
class MapAsImageProvider
{
public:
  MapAsImageProvider()
    : pn_("~")
  {

    image_transport_ = new image_transport::ImageTransport(n_);
    image_transport_publisher_full_ = image_transport_->advertise("map_image/full", 1);
    image_transport_publisher_tile_ = image_transport_->advertise("map_image/tile", 1);

    pose_sub_ = n_.subscribe("pose", 1, &MapAsImageProvider::poseCallback, this);
    map_sub_ = n_.subscribe("map", 1, &MapAsImageProvider::mapCallback, this);

    //Which frame_id makes sense?
    cv_img_full_.header.frame_id = "map_image";
    cv_img_full_.encoding = sensor_msgs::image_encodings::MONO8;

    cv_img_tile_.header.frame_id = "map_image";
    cv_img_tile_.encoding = sensor_msgs::image_encodings::MONO8;

    //Fixed cell width for tile based image, use dynamic_reconfigure for this later
    p_size_tiled_map_image_x_ = 64;
    p_size_tiled_map_image_y_ = 64;

    ROS_INFO("Map to Image node started.");
  }

  ~MapAsImageProvider()
  {
    delete image_transport_;
  }

  //We assume the robot position is available as a PoseStamped here (querying tf would be the more general option)
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose)
  {
    pose_ptr_ = pose;
  }

  //The map->image conversion runs every time a new map is received at the moment
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
  {
    int size_x = map->info.width;
    int size_y = map->info.height;

    if ((size_x < 3) || (size_y < 3) ){
      ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
      return;
    }

    // Only if someone is subscribed to it, do work and publish full map image
    if (image_transport_publisher_full_.getNumSubscribers() > 0){
      cv::Mat* map_mat  = &cv_img_full_.image;

      // resize cv image if it doesn't have the same dimensions as the map
      if ( (map_mat->rows != size_y) && (map_mat->cols != size_x)){
        *map_mat = cv::Mat(size_y, size_x, CV_8U);
      }

      const std::vector<int8_t>& map_data (map->data);

      unsigned char *map_mat_data_p=(unsigned char*) map_mat->data;

      //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
      int size_y_rev = size_y-1;

      for (int y = size_y_rev; y >= 0; --y){

        int idx_map_y = size_x * (size_y_rev - y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x){

          int idx = idx_img_y + x;

          switch (map_data[idx_map_y + x])
          {
          case -1:
            map_mat_data_p[idx] = 127;
            break;

          case 0:
            map_mat_data_p[idx] = 255;
            break;

          case 100:
            map_mat_data_p[idx] = 0;
            break;
          }
        }
      }
      image_transport_publisher_full_.publish(cv_img_full_.toImageMsg());
    }

    // Only if someone is subscribed to it, do work and publish tile-based map image Also check if pose_ptr_ is valid
    if ((image_transport_publisher_tile_.getNumSubscribers() > 0) && (pose_ptr_)){

      world_map_transformer_.setTransforms(*map);

      Eigen::Vector2f rob_position_world (pose_ptr_->pose.position.x, pose_ptr_->pose.position.y);
      Eigen::Vector2f rob_position_map (world_map_transformer_.getC2Coords(rob_position_world));

      Eigen::Vector2i rob_position_mapi (rob_position_map.cast<int>());

      Eigen::Vector2i tile_size_lower_halfi (p_size_tiled_map_image_x_ / 2, p_size_tiled_map_image_y_ / 2);

      Eigen::Vector2i min_coords_map (rob_position_mapi - tile_size_lower_halfi);

      //Clamp to lower map coords
      if (min_coords_map[0] < 0){
        min_coords_map[0] = 0;
      }

      if (min_coords_map[1] < 0){
        min_coords_map[1] = 0;
      }

      Eigen::Vector2i max_coords_map (min_coords_map + Eigen::Vector2i(p_size_tiled_map_image_x_,p_size_tiled_map_image_y_));

      //Clamp to upper map coords
      if (max_coords_map[0] > size_x){

        int diff = max_coords_map[0] - size_x;
        min_coords_map[0] -= diff;

        max_coords_map[0] = size_x;
      }

      if (max_coords_map[1] > size_y){

        int diff = max_coords_map[1] - size_y;
        min_coords_map[1] -= diff;

        max_coords_map[1] = size_y;
      }

      //Clamp lower again (in case the map is smaller than the selected visualization window)
      if (min_coords_map[0] < 0){
        min_coords_map[0] = 0;
      }

      if (min_coords_map[1] < 0){
        min_coords_map[1] = 0;
      }

      Eigen::Vector2i actual_map_dimensions(max_coords_map - min_coords_map);

      cv::Mat* map_mat  = &cv_img_tile_.image;

      // resize cv image if it doesn't have the same dimensions as the selected visualization window
      if ( (map_mat->rows != actual_map_dimensions[0]) || (map_mat->cols != actual_map_dimensions[1])){
        *map_mat = cv::Mat(actual_map_dimensions[0], actual_map_dimensions[1], CV_8U);
      }

      const std::vector<int8_t>& map_data (map->data);

      unsigned char *map_mat_data_p=(unsigned char*) map_mat->data;

      //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
      int y_img = max_coords_map[1]-1;

      for (int y = min_coords_map[1]; y < max_coords_map[1];++y){

        int idx_map_y = y_img-- * size_x;
        int idx_img_y = (y-min_coords_map[1]) * actual_map_dimensions.x();

        for (int x = min_coords_map[0]; x < max_coords_map[0];++x){

          int img_index = idx_img_y + (x-min_coords_map[0]);

          switch (map_data[idx_map_y+x])
          {
          case 0:
            map_mat_data_p[img_index] = 255;
            break;

          case -1:
            map_mat_data_p[img_index] = 127;
            break;

          case 100:
            map_mat_data_p[img_index] = 0;
            break;
          }
        }        
      }
      image_transport_publisher_tile_.publish(cv_img_tile_.toImageMsg());
    }
  }

  ros::Subscriber map_sub_;
  ros::Subscriber pose_sub_;

  image_transport::Publisher image_transport_publisher_full_;
  image_transport::Publisher image_transport_publisher_tile_;

  image_transport::ImageTransport* image_transport_;

  geometry_msgs::PoseStampedConstPtr pose_ptr_;

  cv_bridge::CvImage cv_img_full_;
  cv_bridge::CvImage cv_img_tile_;

  ros::NodeHandle n_;
  ros::NodeHandle pn_;

  int p_size_tiled_map_image_x_;
  int p_size_tiled_map_image_y_;

  HectorMapTools::CoordinateTransformer<float> world_map_transformer_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_to_image_node");

  MapAsImageProvider map_image_provider;

  ros::spin();

  return 0;
}
