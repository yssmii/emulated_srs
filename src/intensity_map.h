// -*- C++ -*-
/*!
 * @file  intensity_map.h
 * @brief Class definition of vbpd_ros::IntensityMapMask
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2019  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifndef __ROS_ESRS_INTENSITY_MAP_H__
#define __ROS_ESRS_INTENSITY_MAP_H__

#include <UFV/ImageData.h>
#include <UFV/IntensityMap.h>

namespace emulated_srs
{

class IntensityMapMask : public UFV::IntensityMap
{
public:
  IntensityMapMask(void) {};
  IntensityMapMask(const int width, const int height)
      : UFV::IntensityMap(width, height, 3, TYPE_U8)
    {
    };
  ~IntensityMapMask(void) {};
  
public:
  virtual void reshape(const int width, const int height) {
    this->UFV::IntensityMap::reshape(width, height, 3, TYPE_U8);
    this->resetMaskImage();
    return;
  };
  
public:
  void detect(void) {};

public:
  int mask(void);

  bool hasMaskImage(void) const { return !mask_image_.isEmpty(); };
  int setMaskImage(const UFV::ImageData<unsigned char> &mask);
  int setMaskImage(const std::string &pathtofile);
  void resetMaskImage(void);
 
protected:
  UFV::ImageData<unsigned char> mask_image_;
};

} // end of namespace emulated_srs

#endif // __ROS_ESRS_YOLOMAPMASK_H__
