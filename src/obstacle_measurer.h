// -*- C++ -*-
/*!
 * @file  obstacle_measurer.h
 * @brief emulated_srs::ObstacleMeasurer class definition
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2021  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __ROS_EMULATED_SRS_OBSTACLE_MEASURER_H__
#define __ROS_EMULATED_SRS_OBSTACLE_MEASURER_H__

#include "eSRS/Object.h"

#include "obstacle_detector.h"

namespace emulated_srs
{

/*!
 * @class ObstacleMeasurer
 *
 * @brief ROS node for object detection and distance measurement
 *
 * - Subscribe PointCloud2 (organized)
 * - Publish information of obstacles detected and measured
 *
 */
class ObstacleMeasurer : public ObstacleDetector
{
public:
  ObstacleMeasurer(void);
  virtual ~ObstacleMeasurer(void) {};

private:
  void initializeMap(const int width,
                     const int height,
                     const int point_step);
  void publishExpSetup(void);
  void displayAll(void);

  int publishObstaclesMessage(
    const std::vector<eSRS::ObstacleClassified> &obs,
    const int nobs);

  void
  setMeasurableInfo(const int object_count,
                    std::vector<emulated_srs::Obstacle> &obsmsgary);

private:
  float param_dist_testpiece_; //!< exp. setup

  int param_use_correct_region_p_;
  std::string param_fname_correct_region_;
  UFV::ImageData<unsigned char> map_correct_region_; //!< pre-measured
  
private:  
  UFV::ImageData<unsigned char> map_for_showing_obstacles_within_;

//private:  
  //std::vector<emulated_srs::ObstacleWithin>  obstacles;

}; //End of class SubscribeAndPublish

} // end of namespace emulated_srs

#endif // __ROS_EMULATED_SRS_OBSTACLE_MEASURER_H__
