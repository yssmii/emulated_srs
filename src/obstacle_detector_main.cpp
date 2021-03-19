// -*- C++ -*-
/**
 * @file  obstacle_detector_main.cpp
 * @brief A ROS node for obstacle detection with classification
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 * 
 * Copyright (C) 2019  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <ros/ros.h>

#include "obstacle_detector.h"

/*!
 *  @brief main function
*/
int main(int argc, char** argv)
{
  ros::init (argc, argv, "obstacle_detector");

  emulated_srs::ObstacleDetector obstacle_detector;

  ros::spin();

  return 0;
}

