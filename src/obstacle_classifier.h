// -*- C++ -*-
/*!
 * @file  obstacle_classifier.h
 * @brief emulated_srs::ObstacleClassifier class definition
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2019  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __ROS_EMULATED_SRS_OBSTACLE_CLASSIFIER_H__
#define __ROS_EMULATED_SRS_OBSTACLE_CLASSIFIER_H__

#include "yolo_map.h"
#include "obstacle_detector.h"

namespace emulated_srs
{

/*!
 * @class ObstacleClassifier
 * @if jp
 *
 * @brief 物体検出及び分類 ROS ノード
 *
 * - PointCloud2 (organized) を subscribe
 * - 検出した障害物情報を publish
 *
 * @endif
 */
class ObstacleClassifier : public ObstacleDetector
{
public:
  ObstacleClassifier(void) {};
  virtual ~ObstacleClassifier(void) {};

private:
  void initializeMap(const int width,
                     const int height);
  int execObstacleDetection(void);
  void displayAll(void);

private:
  //! 障害物分類オブジェクト
  emulated_srs::YoloMapMask map_for_classification_;

}; //End of class SubscribeAndPublish

} // end of namespace emulated_srs

#endif // __ROS_EMULATED_SRS_OBSTACLE_CLASSIFIER_H__
