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

#include "hector_geotiff/geotiff_writer.h"

#include <cstdio>
#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/GetMap.h>

#include <QApplication>

using namespace std;

namespace hector_geotiff{

/**
 * @brief Map generation node.
 */
class MapGenerator
{
  public:
    MapGenerator(const std::string& mapname) : mapname_(mapname)
    {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      ros::Time start_time (ros::Time::now());

      geotiff_writer.setMapFileName(mapname_);
      geotiff_writer.setupTransforms(*map);
      geotiff_writer.drawBackgroundCheckerboard();
      geotiff_writer.drawMap(*map);
      geotiff_writer.drawCoords();

      geotiff_writer.writeGeotiffImage(true);

      ros::Duration elapsed_time (ros::Time::now() - start_time);
      ROS_INFO("GeoTiff created in %f seconds", elapsed_time.toSec());
    }

    GeotiffWriter geotiff_writer;

    std::string mapname_;
    ros::Subscriber map_sub_;
};

}

#define USAGE "Usage: \n" \
              "  geotiff_saver -h\n"\
              "  geotiff_saver [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  std::string mapname = "map";

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  //GeotiffWriter geotiff_writer;
  //geotiff_writer.setMapName("test");
  hector_geotiff::MapGenerator mg(mapname);

  ros::spin();

  return 0;
}

