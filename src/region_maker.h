// -*- C++ -*-
/*!
 * @file  region_maker.h
 * @brief emulated_srs::RegionMaker class definition
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2021  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __ROS_EMULATED_SRS_REGION_MAKER_H__
#define __ROS_EMULATED_SRS_REGION_MAKER_H__

#include <string>

#include <eSRS/Object.h>
#include <emulated_srs/Obstacle.h>

#include "obstacle_detector.h"

namespace emulated_srs
{

/*!
 * @class RegionMaker
 *
 * @brief ROS node to make region for obstacle measurment
 *
 * - Subscribe PointCloud2 (organized)
 * - Save a PNG binary image representing the correct obstacle region
 *
 */
class RegionMaker : public ObstacleDetector
{
public:
  RegionMaker(void);
  virtual ~RegionMaker(void) {};

private:
  void initializeMap(const int width,
                     const int height,
                     const int point_step);
  void displayAll(void);

  int save() noexcept;

private:
  void publishExpSetup(void) {};
  
  int publishObstaclesMessage(
    const std::vector<eSRS::ObstacleClassified> &obs,
    const int nobs) { return UFV::OK; };

  void setMeasurableInfo(const int object_count,
                         std::vector<emulated_srs::Obstacle> &obsmsgary) {};

  int publishMarkersMessage(
    const std::vector<eSRS::ObstacleClassified> &obs,
    const int nobs) { return UFV::OK; };

  int publishImagesMessage(void) { return UFV::OK; };
  
private:
  std::string param_fname_to_save_;
  std::string param_path_to_save_;
}; //End of class SubscribeAndPublish

} // end of namespace emulated_srs

#endif // __ROS_EMULATED_SRS_REGION_MAKER_H__
