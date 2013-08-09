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

#include <ros/console.h>
#include <hector_geotiff/geotiff_writer.h>

#include <QtGui/QPainter>
#include <QtGui/QImageWriter>
#include <QtGui/QApplication>
#include <QtCore/QFile>
//#include <QtCore/QDateTime>
#include <QtCore/QTime>
#include <QtCore/QTextStream>

namespace hector_geotiff{


GeotiffWriter::GeotiffWriter(bool useCheckerboardCacheIn)
  : useCheckerboardCache(useCheckerboardCacheIn)
  , use_utc_time_suffix_(true)
{
  cached_map_meta_data_.height = -1;
  cached_map_meta_data_.width = -1;
  cached_map_meta_data_.resolution = -1.0f;

  fake_argc_ = 0;

  //Create a QApplication cause otherwise drawing text will crash
  app = new QApplication(fake_argc_, fake_argv_, false);

  map_file_name_ = "";
  map_file_path_ = "";
}

GeotiffWriter::~GeotiffWriter()
{
  delete app;
}

void GeotiffWriter::setMapFileName(const std::string& mapFileName)
{
  map_file_name_ = mapFileName;

  if (use_utc_time_suffix_){
    //QDateTime now (QDateTime::currentDateTimeUtc());
    //std::string current_time_string = now.toString(Qt::ISODate).toStdString();
    QTime now (QTime::currentTime());
    std::string current_time_string = now.toString(Qt::ISODate).toStdString();

    map_file_name_ += "_" + current_time_string;
  }
}

void GeotiffWriter::setMapFilePath(const std::string& mapFilePath)
{
  map_file_path_ = mapFilePath;
}

void GeotiffWriter::setUseUtcTimeSuffix(bool useSuffix)
{
  use_utc_time_suffix_ = useSuffix;
}


bool GeotiffWriter::setupTransforms(const nav_msgs::OccupancyGrid& map)
{
  resolution = static_cast<float>(map.info.resolution);
  origin = Eigen::Vector2f(map.info.origin.position.x, map.info.origin.position.y);

  resolutionFactor = 3;
  resolutionFactorf = static_cast<float>(resolutionFactor);

  pixelsPerMapMeter = 1.0f / map.info.resolution;
  pixelsPerGeoTiffMeter = pixelsPerMapMeter * static_cast<float>(resolutionFactor);

  minCoordsMap = Eigen::Vector2i::Zero();
  maxCoordsMap = Eigen::Vector2i(map.info.width, map.info.height);

  if(!HectorMapTools::getMapExtends(map, minCoordsMap, maxCoordsMap)){
    ROS_INFO("Cannot determine map extends!");
    return false;
  }

  sizeMap = Eigen::Vector2i(maxCoordsMap - minCoordsMap);
  sizeMapf = ((maxCoordsMap - minCoordsMap).cast<float>());


  rightBottomMarginMeters = Eigen::Vector2f(1.0f, 1.0f);
  rightBottomMarginPixelsf = Eigen::Vector2f( rightBottomMarginMeters.array() * pixelsPerGeoTiffMeter);
  rightBottomMarginPixels = ((rightBottomMarginPixelsf.array() +0.5f).cast<int>());

  leftTopMarginMeters= Eigen::Vector2f(3.0f, 3.0f);

  totalMeters = (rightBottomMarginMeters + sizeMapf* map.info.resolution + leftTopMarginMeters);
  //std::cout << "\n" << totalMeters;

  totalMeters.x() = ceil(totalMeters.x());
  totalMeters.y() = ceil(totalMeters.y());
  //std::cout << "\n" << totalMeters;

  geoTiffSizePixels = ( (totalMeters.array() * pixelsPerGeoTiffMeter).cast<int>());


  mapOrigInGeotiff = (rightBottomMarginPixelsf);
  mapEndInGeotiff = (rightBottomMarginPixelsf + sizeMapf * resolutionFactorf);
  //std::cout << "\n mapOrig\n" << mapOrigInGeotiff;
  //std::cout << "\n mapOrig\n" << mapEndInGeotiff;

  world_map_transformer_.setTransforms(map);

  map_geo_transformer_.setTransformsBetweenCoordSystems(mapOrigInGeotiff,mapEndInGeotiff, minCoordsMap.cast<float>(),maxCoordsMap.cast<float>());

  /*
  Eigen::Vector2f temp_zero_map_g (map_geo_transformer_.getC2Coords(Eigen::Vector2f::Zero()));

  Eigen::Vector2f temp_zero_map_g_floor (floor(temp_zero_map_g.x()), floor(temp_zero_map_g.x()));

  Eigen::Vector2f diff (temp_zero_map_g - temp_zero_map_g_floor);

  map*/


  Eigen::Vector2f p1_w (Eigen::Vector2f::Zero());
  Eigen::Vector2f p2_w (Eigen::Vector2f(100.0f, 100.0f));

  Eigen::Vector2f p1_m (world_map_transformer_.getC2Coords(p1_w));
  Eigen::Vector2f p2_m (world_map_transformer_.getC2Coords(p2_w));

  Eigen::Vector2f p1_g (map_geo_transformer_.getC2Coords(p1_m));
  Eigen::Vector2f p2_g (map_geo_transformer_.getC2Coords(p2_m));

  world_geo_transformer_.setTransformsBetweenCoordSystems(p1_g, p2_g, p1_w, p2_w);

  map_draw_font_ = QFont();
  map_draw_font_.setPixelSize(6*resolutionFactor);

  if (useCheckerboardCache){

    if ((cached_map_meta_data_.height != map.info.height) ||
        (cached_map_meta_data_.width != map.info.width) ||
        (cached_map_meta_data_.resolution = map.info.resolution)){

      cached_map_meta_data_ = map.info;

      Eigen::Vector2f img_size (Eigen::Vector2f(map.info.width,map.info.height)* resolutionFactorf + (rightBottomMarginMeters + leftTopMarginMeters)*pixelsPerGeoTiffMeter );
      checkerboard_cache = QImage(img_size.y(),img_size.x(), QImage::Format_RGB32);

      QPainter qPainter(&image);
      transformPainterToImgCoords(qPainter);

      QBrush c1 = QBrush(QColor(226, 226, 227));
      QBrush c2 = QBrush(QColor(237, 237, 238));
      QRectF background_grid_tile(0.0f, 0.0f, pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter);


      int xMaxGeo = geoTiffSizePixels[0];
      int yMaxGeo = geoTiffSizePixels[1];

      for (int y = 0; y < yMaxGeo; ++y){
        for (int x = 0; x < xMaxGeo; ++x){
          //std::cout << "\n" << x << " " << y;

          if ((x + y) % 2 == 0) {
            //qPainter.fillRect(background_grid_tile, c1);
            qPainter.fillRect(static_cast<float>(x)*pixelsPerGeoTiffMeter,static_cast<float>(y)*pixelsPerGeoTiffMeter,pixelsPerGeoTiffMeter,pixelsPerGeoTiffMeter, c1);

          } else {
            //qPainter.fillRect(background_grid_tile, c2);
            qPainter.fillRect(static_cast<float>(x)*pixelsPerGeoTiffMeter,static_cast<float>(y)*pixelsPerGeoTiffMeter,pixelsPerGeoTiffMeter,pixelsPerGeoTiffMeter, c2);
          }
          //background_grid_tile.moveTo(QPointF(static_cast<float>(x)*pixelsPerGeoTiffMeter,static_cast<float>(y)*pixelsPerGeoTiffMeter));
        }
      }

    }

  }

  return true;
}

void GeotiffWriter::setupImageSize()
{
  bool painter_rotate = true;

  int xMaxGeo = geoTiffSizePixels[0];
  int yMaxGeo = geoTiffSizePixels[1];

  if (!useCheckerboardCache){
    if (painter_rotate){
      image = QImage(yMaxGeo, xMaxGeo, QImage::Format_RGB32);
    }else{
      image = QImage(xMaxGeo, yMaxGeo, QImage::Format_RGB32);
    }

    QPainter qPainter(&image);

    QBrush grey = QBrush(QColor(128, 128, 128));

    qPainter.fillRect(image.rect(), grey);

  }
}

void GeotiffWriter::drawBackgroundCheckerboard()
{
  int xMaxGeo = geoTiffSizePixels[0];
  int yMaxGeo = geoTiffSizePixels[1];

  bool painter_rotate = true;

  if (!useCheckerboardCache){

    QPainter qPainter(&image);

    if (painter_rotate){
      transformPainterToImgCoords(qPainter);
    }

    //*********************** Background checkerboard pattern **********************
    QBrush c1 = QBrush(QColor(226, 226, 227));
    QBrush c2 = QBrush(QColor(237, 237, 238));
    QRectF background_grid_tile(0.0f, 0.0f, pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter);



    for (int y = 0; y < yMaxGeo; ++y){
      for (int x = 0; x < xMaxGeo; ++x){
        //std::cout << "\n" << x << " " << y;

        if ((x + y) % 2 == 0) {
          //qPainter.fillRect(background_grid_tile, c1);
          qPainter.fillRect(static_cast<float>(x)*pixelsPerGeoTiffMeter,static_cast<float>(y)*pixelsPerGeoTiffMeter,pixelsPerGeoTiffMeter,pixelsPerGeoTiffMeter, c1);

        } else {
          //qPainter.fillRect(background_grid_tile, c2);
          qPainter.fillRect(static_cast<float>(x)*pixelsPerGeoTiffMeter,static_cast<float>(y)*pixelsPerGeoTiffMeter,pixelsPerGeoTiffMeter,pixelsPerGeoTiffMeter, c2);
        }
        //background_grid_tile.moveTo(QPointF(static_cast<float>(x)*pixelsPerGeoTiffMeter,static_cast<float>(y)*pixelsPerGeoTiffMeter));
      }
    }
  }else{
    image = checkerboard_cache.copy(0,0,geoTiffSizePixels[0],geoTiffSizePixels[1]);
  }
}

void GeotiffWriter::drawMap(const nav_msgs::OccupancyGrid& map, bool draw_explored_space_grid)
{
  QPainter qPainter(&image);

  transformPainterToImgCoords(qPainter);

  //this->drawCoordSystem(qPainter);

  QRectF map_cell_grid_tile(0.0f, 0.0f, resolutionFactor, resolutionFactor);

  QBrush occupied_brush(QColor(0, 40, 120));
  QBrush free_brush(QColor(255, 255, 255));
  QBrush explored_space_grid_brush(QColor(190,190,191));

  int width = map.info.width;

  float explored_space_grid_resolution_pixels = pixelsPerGeoTiffMeter * 0.5f;

  float yGeo = 0.0f;
  float currYLimit = 0.0f;

  bool drawY = false;

  for (int y = minCoordsMap[1] ; y < maxCoordsMap[1]; ++y){

    float xGeo = 0.0f;

    if(yGeo >= currYLimit ){
      drawY = true;
    }

    float currXLimit = 0.0f;
    bool drawX = false;

    for (int x = minCoordsMap[0] ; x < maxCoordsMap[0]; ++x){

      unsigned int i = y*width + x;

      int8_t data = map.data[i];

      if (xGeo >= currXLimit){
        drawX = true;
      }

      if (data == 0){

        Eigen::Vector2f coords(mapOrigInGeotiff + (Eigen::Vector2f(xGeo,yGeo)));
        qPainter.fillRect(coords[0],coords[1],resolutionFactorf, resolutionFactorf, free_brush);


        if (draw_explored_space_grid){
          if (drawY){
            qPainter.fillRect(coords[0],mapOrigInGeotiff.y() + currYLimit, resolutionFactorf, 1.0f, explored_space_grid_brush);
          }

          if (drawX){
            qPainter.fillRect(mapOrigInGeotiff.x() + currXLimit, coords[1], 1.0f, resolutionFactorf, explored_space_grid_brush);
          }
        }

      }else if(data == 100){
        qPainter.fillRect(mapOrigInGeotiff.x()+xGeo, mapOrigInGeotiff.y()+yGeo,resolutionFactorf, resolutionFactorf, occupied_brush);
      }

      if(drawX){
        currXLimit += explored_space_grid_resolution_pixels;
        drawX=false;
      }

      xGeo+=resolutionFactorf;
    }

    if(drawY){
      drawY=false;
      currYLimit += explored_space_grid_resolution_pixels;
    }

    yGeo+= resolutionFactorf;
  }
}

void GeotiffWriter::drawObjectOfInterest(const Eigen::Vector2f& coords, const std::string& txt, const Color& color)
{
  QPainter qPainter(&image);

  transformPainterToImgCoords(qPainter);


  //qPainter.setPen(ellipse_pen_);

  Eigen::Vector2f map_coords  (world_map_transformer_.getC2Coords(coords) );

  Eigen::Vector2f coords_g (world_geo_transformer_.getC2Coords(coords));

  qPainter.translate(coords_g[0],coords_g[1]);

  qPainter.rotate(90);

  qPainter.setRenderHint(QPainter::Antialiasing, true);

  float radius = pixelsPerGeoTiffMeter * 0.175f;

  QRectF ellipse_shape( - radius, - radius, radius*2.0f, radius*2.0f);
  qPainter.save();


  QBrush tmpBrush(QColor(color.r,color.g,color.b));
  QPen tmpPen(Qt::NoPen);
  qPainter.setBrush(tmpBrush);
  qPainter.setPen(tmpPen);

  qPainter.drawEllipse(ellipse_shape);
  qPainter.restore();


  QString tmp (txt.c_str());
  //tmp.setNum(number);

  if (tmp.length() < 2){
    qPainter.setFont(map_draw_font_);
  }else{
    QFont tmp_font;
    tmp_font.setPixelSize(3*resolutionFactor);
    qPainter.setFont(tmp_font);
  }



  qPainter.setPen(Qt::white);
  qPainter.scale(-1.0,1.0);

  qPainter.drawText(ellipse_shape,Qt::AlignCenter , tmp);
}

void GeotiffWriter::drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points)
{
  QPainter qPainter(&image);

  transformPainterToImgCoords(qPainter);

  Eigen::Vector2f start_geo (world_geo_transformer_.getC2Coords(start.head<2>()));



  size_t size = points.size();

  QPolygonF polygon;
  polygon.reserve(size);

  polygon.push_back(QPointF(start_geo.x(), start_geo.y()));

  for (size_t i = 0; i < size; ++i){
    const Eigen::Vector2f vec (world_geo_transformer_.getC2Coords(points[i]));
    polygon.push_back(QPointF(vec.x(), vec.y()));
  }

  QPen pen(qPainter.pen());
  pen.setColor(QColor(120,0,240));
  pen.setWidth(3);

  qPainter.setPen(pen);

  //qPainter.setPen(QColor(120,0,240));


  qPainter.drawPolyline(polygon);

  qPainter.save();
  qPainter.translate(start_geo.x(), start_geo.y());
  qPainter.rotate(start.z());
  qPainter.setRenderHint(QPainter::Antialiasing, true);
  drawArrow(qPainter);
  //drawCoordSystem(qPainter);
  qPainter.restore();
}

std::string GeotiffWriter::getBasePathAndFileName() const
{
  return std::string (map_file_path_ +"/" + map_file_name_);
}

void GeotiffWriter::writeGeotiffImage()
{
  //Only works with recent Qt versions
  //QDateTime now (QDateTime::currentDateTimeUtc());
  //std::string current_time_string = now.toString(Qt::ISODate).toStdString();


  std::string complete_file_string ( map_file_path_ +"/" + map_file_name_ +".tif");
  QImageWriter imageWriter(QString::fromStdString(complete_file_string));
  imageWriter.setCompression(1);

  bool success = imageWriter.write(image);

  std::string tfw_file_name (map_file_path_ +"/" + map_file_name_ + ".tfw");
  QFile tfwFile(QString::fromStdString(tfw_file_name));

  tfwFile.open(QIODevice::WriteOnly);

  QTextStream out(&tfwFile);

  float resolution_geo = resolution / resolutionFactorf;

  QString resolution_string;
  resolution_string.setNum(resolution_geo,'f',10);

  //positive x resolution
  out << resolution_string << "\n";

  QString zero_string;
  zero_string.setNum(0.0f, 'f', 10);

  //rotation, translation
  out << zero_string << "\n" << zero_string << "\n";

  //negative y resolution
  out << "-" << resolution_string << "\n";

  QString top_left_string_x;
  QString top_left_string_y;

  //Eigen::Vector2f zero_map_w = world_map_transformer_.getC1Coords(Eigen::Vector2f::Zero());
  Eigen::Vector2f zero_geo_w (world_geo_transformer_.getC1Coords((geoTiffSizePixels.array()+1).cast<float>()));


  top_left_string_x.setNum(-zero_geo_w.y(),'f',10);
  top_left_string_y.setNum(zero_geo_w.x(),'f',10);

  out << top_left_string_x << "\n" << top_left_string_y << "\n";

  tfwFile.close();

  if(!success){
    ROS_INFO("Writing image with file %s failed with error %s", complete_file_string.c_str(), imageWriter.errorString().toStdString().c_str());
  }else{
    ROS_INFO("Successfully wrote geotiff to %s", complete_file_string.c_str());
  }
}

void GeotiffWriter::transformPainterToImgCoords(QPainter& painter)
{
  painter.rotate(-90);
  painter.translate(-geoTiffSizePixels.x(),geoTiffSizePixels.y());
  painter.scale(1.0,-1.0);
}

void GeotiffWriter::drawCoords()
{
  QPainter qPainter(&image);
  qPainter.setFont(map_draw_font_);

  float arrowOffset = pixelsPerGeoTiffMeter * 0.15f;

  // MAP ORIENTATION
  qPainter.setPen(QColor(0, 50, 140));
  qPainter.drawLine(pixelsPerGeoTiffMeter / 2, pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter / 2, 2.0f * pixelsPerGeoTiffMeter);
  qPainter.drawLine(pixelsPerGeoTiffMeter * 2 / 5, pixelsPerGeoTiffMeter - 1, pixelsPerGeoTiffMeter * 3 / 5, pixelsPerGeoTiffMeter - 1);
  qPainter.drawLine(pixelsPerGeoTiffMeter * 2 / 5, 2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter * 3 / 5, 2 * pixelsPerGeoTiffMeter);



  qPainter.drawLine(pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter);
  qPainter.drawLine(pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter + arrowOffset, 2 * pixelsPerGeoTiffMeter - arrowOffset);
  qPainter.drawLine(pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter + arrowOffset, 2 * pixelsPerGeoTiffMeter + arrowOffset);

  qPainter.drawLine(2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter);
  qPainter.drawLine(2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter + arrowOffset, pixelsPerGeoTiffMeter + arrowOffset);
  qPainter.drawLine(2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter - arrowOffset, pixelsPerGeoTiffMeter + arrowOffset);

  qPainter.drawText(0.6 * pixelsPerGeoTiffMeter, 1.6 * pixelsPerGeoTiffMeter, QString("1m"));

  qPainter.drawText(2.2 * pixelsPerGeoTiffMeter, 1.1 * pixelsPerGeoTiffMeter, QString("x"));
  qPainter.drawText(1.2 * pixelsPerGeoTiffMeter, 1.8 * pixelsPerGeoTiffMeter, QString("y"));

  qPainter.drawText (0.5f*pixelsPerGeoTiffMeter,0.75f*pixelsPerGeoTiffMeter, QString((map_file_name_ + ".tif").c_str()));
}

void GeotiffWriter::drawCross(QPainter& painter, const Eigen::Vector2f& coords)
{
  painter.drawLine(QPointF(coords[0]-1.0f, coords[1]), QPointF(coords[0]+1.0f, coords[1]));
  painter.drawLine(QPointF(coords[0]  , coords[1]-1.0f), QPointF(coords[0], coords[1]+1.0f));
}

void GeotiffWriter::drawArrow(QPainter& painter)
{
  float tip_distance = pixelsPerGeoTiffMeter * 0.3f;

  QPolygonF polygon;

  polygon << QPointF(tip_distance, 0.0f) << QPointF(-tip_distance*0.5f, -tip_distance*0.5f) << QPointF(0.0f, 0.0f) << QPointF(-tip_distance*0.5f, tip_distance*0.5f);

  painter.save();

  QBrush tmpBrush(QColor(255,200,0));
  QPen tmpPen(Qt::NoPen);
  painter.setBrush(tmpBrush);
  painter.setPen(tmpPen);

  painter.drawPolygon(polygon);

  painter.restore();
}

void GeotiffWriter::drawCoordSystem(QPainter& painter)
{
  painter.save();
  QPointF zero_point (0.0f, 0.0f);
  QPointF x_point (pixelsPerGeoTiffMeter, 0.0f);
  QPointF y_point (0.0f, pixelsPerGeoTiffMeter);

  QPen tmp = painter.pen();
  tmp.setWidth(5);
  tmp.setColor(QColor(255.0,0.0,0.0));
  //painter.setPen(QPen::setWidth(5));
  painter.setPen(tmp);
  painter.drawLine(zero_point,x_point);

  tmp.setColor(QColor(0,255,0));
  painter.setPen(tmp);
  painter.drawLine(zero_point,y_point);

  painter.restore();
}

}
