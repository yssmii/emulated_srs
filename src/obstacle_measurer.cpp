// -*- C++ -*-
/**
 * @file  obstacle_measurer.cpp
 * @brief A ROS node for obstacle detection and distance measurement
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 * 
 * Copyright (C) 2021  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <ros/ros.h>

#include "obstacle_measurer.h"

/*!
 *  @brief main function
*/
int main(int argc, char** argv)
{
  ros::init (argc, argv, "obstacle_measurer");

  emulated_srs::ObstacleMeasurer obstacle_measurer;

  ros::spin();

  return 0;
}

void emulated_srs::ObstacleMeasurer::initializeMap(
  const int width,
  const int height)
{
  this->emulated_srs::ObstacleDetector::initializeMap(width,height);

  return;
}

/*!
*/
int emulated_srs::ObstacleMeasurer::execObstacleDetection(void)
{
  int object_count = this->emulated_srs::ObstacleDetector::execObstacleDetection();

  return object_count;
}

void emulated_srs::ObstacleMeasurer::displayAll(void)
{
  this->emulated_srs::ObstacleDetector::displayAll();

  return;
  
}
