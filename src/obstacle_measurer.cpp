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

#include <emulated_srs/Obstacle.h>
#include <emulated_srs/ExpSetup.h>

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

emulated_srs::ObstacleMeasurer::ObstacleMeasurer(void)
    :
    emulated_srs::ObstacleDetector(),
    param_dist_testpiece_(1500.0),
    param_use_correct_region_p_(0),
    param_fname_correct_region_("Reg.png"),
    min_z_(0),
    max_z_(0)
{
  node_handle_.getParam("dist_testpiece", param_dist_testpiece_);
  node_handle_.getParam("use_region_p", param_use_correct_region_p_);
  node_handle_.getParam("filename_region", param_fname_correct_region_);

  ROS_INFO("dist_testpiece: %f", param_dist_testpiece_);
  ROS_INFO("use_region_p: %d", param_use_correct_region_p_);
  ROS_INFO("filename_region: %s", param_fname_correct_region_.c_str());
}

void emulated_srs::ObstacleMeasurer::initializeMap(
  const int width,
  const int height,
  const int point_step)
{
  this->emulated_srs::ObstacleDetector::initializeMap(width,height,point_step);

  if(param_use_correct_region_p_)
  {
    cv::Mat corimg = cv::imread(param_fname_correct_region_.c_str(), 0);
    if(corimg.data == NULL)
    {
      ROS_ERROR("Cannot open region image: %s",
                param_fname_correct_region_.c_str());
      return;
    }

    cv::Size size = corimg.size();
  
    map_correct_region_.setData(size.width, size.height, 1, corimg.ptr());
    map_for_showing_obstacles_within_.reshape(size.width, size.height, 3);
  }
  
  return;
}

void emulated_srs::ObstacleMeasurer::publishExpSetup(void)
{
  // publish the experimental setup as a latch topic
  emulated_srs::ExpSetup exp_setup;
  
  exp_setup.name_sensor = param_name_sensor_;
  exp_setup.dist_testpiece = param_dist_testpiece_;
  exp_setup.fname_mask = param_use_mask_p_ ? param_fname_mask_ : "";
  exp_setup.param_zkey = param_zkey_;
  exp_setup.param_min_gap = param_gap_;
  exp_setup.param_min_size = param_min_size_;

  exp_setup.fname_region =
    param_use_correct_region_p_ ? param_fname_correct_region_ : "";

  publisher_exp_setup_.publish(exp_setup);

  return;
}


void
emulated_srs::ObstacleMeasurer::setMeasurableInfo(
  const int object_count,
  std::vector<emulated_srs::Obstacle> &obsmsgary)
{
  if(object_count <= 0) return;
  if((long unsigned int)object_count != obsmsgary.size())
  {
    ROS_WARN("Num of the detected obstacles mismatch: %d != %lu",
             object_count, obsmsgary.size());
    return;
  }
  
  UFV::ImageData<short> labeled = map_for_detection_.getLabeledImageData();

  int length = labeled.width()*labeled.height();
  
  short *plbl = labeled.data();
  uchar *pcor = map_correct_region_.data();
  uchar *pshow = map_for_showing_obstacles_within_.data();
  for(int i=0; i<length; i++)
  { 
    pshow[0] = pshow[1] = pshow[2] = 0;
    if(*pcor > 0)
    {
      pshow[2] = 255; // R
      if(*plbl > 0)
      {
        obsmsgary[*plbl - 1].n_points_within += 1;
      }
      else
      {
        pshow[0] = 255;
        pshow[1] = 255;
      }
    }

    plbl++;
    pcor++;
    pshow += 3;
  }
    
  return;
}

int emulated_srs::ObstacleMeasurer::publishObstaclesMessage(
  const std::vector<eSRS::ObstacleClassified> &obstacle_classified,
  const int object_count)
{
  std::vector<emulated_srs::Obstacle> obsmsgary;

  this->emulated_srs::ObstacleDetector::setObstaclesMessage(obstacle_classified, object_count, obsmsgary);

  if(param_use_correct_region_p_ && (object_count > 0))
  {
    setMeasurableInfo(object_count, obsmsgary);
  }

  //! if object_count==0, publish an  empty obstacle
  publisher_obstacle_.publish(obsmsgary[0]);
  for(int i = 1; i < object_count; i++)
  {
    publisher_obstacle_.publish(obsmsgary[i]);
  }

  return UFV::OK;
}

void emulated_srs::ObstacleMeasurer::displayAll(void)
{
  this->emulated_srs::ObstacleDetector::displayAll();

  if (param_display_images_p_ && param_use_correct_region_p_)
  {
    map_for_showing_obstacles_within_.display("Region", 10);
  }

  return;
  
}
