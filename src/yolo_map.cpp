// -*- C++ -*-
/*!
 * @file  yolo_map.cpp
 * @brief 
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

#include <string>

#include <opencv2/highgui/highgui.hpp>

#include <UFV/ImageData.h>
#include "yolo_map.h"

void
emulated_srs::YoloMapMask::resetMaskImage(void)
{
  UFV::ImageData<unsigned char> m;
  mask_image_ = m;

  return;
}

int
emulated_srs::YoloMapMask::setMaskImage(const UFV::ImageData<unsigned char> &mask)
{
  if(mask.width() != this->width() || mask.height() != this->height() ||
     mask.nchannels() != 1)
    return UFV::NG;

  mask_image_ = mask;

  return UFV::OK;
}

int
emulated_srs::YoloMapMask::setMaskImage(const std::string &maskfile)
{
  UFV::ImageData<unsigned char> maskimg;

  cv::Mat mask = cv::imread(maskfile, 0);
  if(mask.data == NULL) return UFV::NG;

  cv::Size size = mask.size();

  maskimg.setData(size.width, size.height, 1, mask.ptr());
  //maskimg.display("mask", 0);
  int ret = this->setMaskImage(maskimg);

  return ret;
}

int
emulated_srs::YoloMapMask::mask(void)
{
  if(!this->hasMaskImage()) return UFV::NG;

  unsigned char *pdat = this->getData<unsigned char>();
  unsigned char *pmask = mask_image_.data();

  for(int i=0; i<this->width()*this->height(); i++)
  {
    if(*pmask == 0)
    {
      *pdat = 0;
      *(pdat+1) = 0;
      *(pdat+2) = 0;
    }
    pmask++;
    pdat+=3;
  }
  
  return UFV::OK;
}
