// -*- C++ -*-
/**
 * @file  region_maker.cpp
 * @brief A ROS node to make correct region for obstacle measurment
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

#include <UFV/types.h>
#include "region_maker.h"

/*!
 *  @brief main function
*/
int main(int argc, char** argv)
{
  ros::init (argc, argv, "region_maker");

  emulated_srs::RegionMaker region_maker;

  ros::spin();

  return 0;
}

emulated_srs::RegionMaker::RegionMaker(void)
    :
    emulated_srs::ObstacleDetector(),
    param_fname_to_save_("Reg.png"),
    param_path_to_save_("./")
{
  node_handle_.getParam("filename_region", param_fname_to_save_);
  node_handle_.getParam("path_to_save", param_path_to_save_);

  ROS_INFO("filename_region: %s", param_fname_to_save_.c_str());
  ROS_INFO("path_to_save: %s", param_path_to_save_.c_str());
}

void emulated_srs::RegionMaker::initializeMap(
  const int width,
  const int height,
  const int point_step)
{
  this->emulated_srs::ObstacleDetector::initializeMap(width,height,point_step);
  map_for_detection_.setDrawLabel(true);
    
  
  return;
}

int
emulated_srs::RegionMaker::save(void) noexcept
{
  int width = map_for_showing_depth_data_.width();
  int height = map_for_showing_depth_data_.height();
  UFV::ImageData<unsigned char> regimg(width, height, 1);
  unsigned char *rimg = regimg.data();
  unsigned char *dimg = map_for_showing_depth_data_.data();

  for(int i=0; i<width*height; i++)
  {
    if(*(dimg+2) > 0  && *dimg ==  0) // R is positive, and B is zero
    {
      *rimg = 255;
    }
    rimg ++;
    dimg += 3;
  }

  regimg.setWriteImage(true, param_path_to_save_);
  regimg.writeImage(param_fname_to_save_);

  ROS_INFO("Saved: %s", (param_path_to_save_ + param_fname_to_save_).c_str());

  return(UFV::OK);
}


void emulated_srs::RegionMaker::displayAll(void)
{
  // Copy the depth image with rtection results for display
  map_for_detection_.normalize(map_for_showing_depth_data_);

  if(has_rgb_data_)
  {
    // Copy the current RGB image.
    map_for_showing_rgb_data_ = *(map_for_rgb_display_.getImageData<unsigned char>());
  }

  // Overwrite the depth image with the obstacle reasons.
  //map_for_detection_.drawObstacleRegionWithLabel(map_for_showing_depth_data_);
  map_for_detection_.setDrawLabel(false);
  map_for_detection_.drawObstacleRegion(map_for_showing_depth_data_);

  UFV::KeyDef dret1, dret2, dret3;
  
  // Display the images.
  if(has_rgb_data_)
  {
    dret1 = map_for_showing_rgb_data_.display("RGB", -1);
  }

  map_for_detection_.setDrawLabel(true);
  dret2 = map_for_detection_.display("Detection",-1);
  dret3 = map_for_showing_depth_data_.display("Depth", 10);

  //std::cout << dret1 << ", " << dret2 << ", " << dret3 << std::endl;

  if(dret1 == UFV::KEY_SAVE || dret2 == UFV::KEY_SAVE ||
     dret3 == UFV::KEY_SAVE)
  {
    this->save();
  }

  return;
  
}
