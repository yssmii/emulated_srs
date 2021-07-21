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
  ObstacleMeasurer(void) {};
  virtual ~ObstacleMeasurer(void) {};

private:
  void initializeMap(const int width,
                     const int height);
  int execObstacleDetection(void);
  void displayAll(void);

}; //End of class SubscribeAndPublish

} // end of namespace emulated_srs

#endif // __ROS_EMULATED_SRS_OBSTACLE_MEASURER_H__
