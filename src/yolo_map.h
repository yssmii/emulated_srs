// -*- C++ -*-
/*!
 * @file  yolo_map.h
 * @brief Class definition of vbpd_ros::YoloMapMask
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2019  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifndef __ROS_ESRS_YOLO_MAP_H__
#define __ROS_ESRS_YOLO_MAP_H__

#include <UFV/ImageData.h>
#include <UFV/IntensityMap.h>
#include <eSRS/Object.h>

#ifdef WITH_YOLO
#include <eSRS/YoloMap.h>
#else
namespace eSRS
{

class YoloMap : public UFV::IntensityMap
{
private:
  
public:
  YoloMap(void) {};
  YoloMap(const int width, const int height)
      : UFV::IntensityMap(width, height, 3, TYPE_U8)
    {
    };
  ~YoloMap(void) {};

public:
  virtual void reshape(const int width, const int height) {
    this->UFV::IntensityMap::reshape(width, height, 3, TYPE_U8);
    return;
  };
  
public:
  void detect(void) {};

public:
  int drawBoundingBox(UFV::ImageData<unsigned char> &bbox) const
    {
      return UFV::OK;
    };
  void getLabeledObject(std::vector<YoloObject> &objs) const
    {
      objs.clear();
    };
};

} // end of namespace eSRS
#endif

namespace emulated_srs
{
 
class YoloMapMask : public eSRS::YoloMap
{
public:
  YoloMapMask(void) {};
  YoloMapMask(const int width, const int height)
      : eSRS::YoloMap(width, height)
    {
    };
  ~YoloMapMask(void) {};
public:
  int mask(void);

  bool hasMaskImage(void) const { return !mask_image_.isEmpty(); };
  int setMaskImage(const UFV::ImageData<unsigned char> &mask);
  int setMaskImage(const std::string &pathtofile);
  void resetMaskImage(void);

  void reshape(const int width, const int height) {
    this->eSRS::YoloMap::reshape(width, height);
    this->resetMaskImage();
    return;
  };

protected:
  UFV::ImageData<unsigned char> mask_image_;
};

} // end of namespace emulated_srs


#endif // __ROS_ESRS_YOLOMAPMASK_H__
