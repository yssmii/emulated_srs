// -*- C++ -*-
/** 
 * @file RangeData.cpp
 * @brief
 *
 * @author Yasushi SUMI (AIST)
 * 
 * Copyright (C) 2018 AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#include "UFV/types.h"
#include "UFV/ImageData.h"
#include "UFV/IntensityMap.h"

using namespace UFV;

IntensityMap::IntensityMap(void)
    : m_prop(PROP_INTENSITY),
      m_type(TYPE_U8),
      m_image(0)      // 0で初期化
{
  //m_image = new UFV::ImageData<unsigned char>(640, 480, 1);
}

IntensityMap::IntensityMap(
  const int width, const int height, const int nchannels,
  Type type)
    : m_prop(PROP_INTENSITY),
      m_type(type)
{
  if(m_type == TYPE_U8)
    m_image = new UFV::ImageData<unsigned char>(width, height, nchannels);
  else if(m_type == TYPE_S16)
    m_image = new UFV::ImageData<short>(width, height, nchannels);
  else if(m_type == TYPE_F32)
    m_image = new UFV::ImageData<float>(width, height, nchannels);
  else
    m_image = new UFV::ImageData<double>(width, height, nchannels);
}

IntensityMap::IntensityMap(const IntensityMap &im)
    : m_prop(im.getProp()),
      m_type(im.getType())
{
  if(m_type == TYPE_U8)
    m_image = new UFV::ImageData<unsigned char>(*(im.getImageData<unsigned char>()));
  else if(m_type == TYPE_S16)
    m_image = new UFV::ImageData<short>(*(im.getImageData<short>()));
  else if(m_type == TYPE_F32)
    m_image = new UFV::ImageData<float>(*(im.getImageData<float>()));
  else
    m_image = new UFV::ImageData<double>(*(im.getImageData<double>()));
}
  
IntensityMap&
IntensityMap::operator=(const IntensityMap &im)
{
  m_prop = im.getProp();

  Type type = im.getType();
  if(m_type == type)
  {
    if(m_type == TYPE_U8)
      *((UFV::ImageData<unsigned char>*)m_image) = *(im.getImageData<unsigned char>());
    else if(m_type == TYPE_S16)
      *((UFV::ImageData<short>*)m_image) = *(im.getImageData<short>());
    else if(m_type == TYPE_F32)
      *((UFV::ImageData<float>*)m_image) = *(im.getImageData<float>());
    else
      *((UFV::ImageData<double>*)m_image) = *(im.getImageData<double>());
  }
  else
  {
    if(m_image != 0) delete m_image;
    m_type = type;
    if(m_type == TYPE_U8)
      m_image = new UFV::ImageData<unsigned char>(*(im.getImageData<unsigned char>()));
    else if(m_type == TYPE_S16)
      m_image = new UFV::ImageData<short>(*(im.getImageData<short>()));
    else if(m_type == TYPE_F32)
      m_image = new UFV::ImageData<float>(*(im.getImageData<float>()));
    else
      m_image = new UFV::ImageData<double>(*(im.getImageData<double>()));
  }

  return *this;
}

void
IntensityMap::reshape(
  const int width, const int height, const int nchannels, Type type)
{
  m_prop =PROP_INTENSITY;
  m_type = type;
  if(m_image != 0) delete m_image;
  
  if(m_type == TYPE_U8)
    m_image = new UFV::ImageData<unsigned char>(width, height, nchannels);
  else if(m_type == TYPE_S16)
    m_image = new UFV::ImageData<short>(width, height, nchannels);
  else if(m_type == TYPE_F32)
    m_image = new UFV::ImageData<float>(width, height, nchannels);
  else
    m_image = new UFV::ImageData<double>(width, height, nchannels);

  return;
}

int
IntensityMap::normalize(
  UFV::ImageData<unsigned char> &ret_img,
  const double maxval) const
{
  int err=UFV::NG;
  
  if(m_image == 0) return err;
  
  if(m_type == TYPE_U8)
    err = ((UFV::ImageData<unsigned char>*)m_image)->normalize(ret_img, maxval);
  else if(m_type == TYPE_S16)
    err = ((UFV::ImageData<short>*)m_image)->normalize(ret_img, maxval);
  else if(m_type == TYPE_F32)
    err = ((UFV::ImageData<float>*)m_image)->normalize(ret_img, maxval);
  else
    err = ((UFV::ImageData<double>*)m_image)->normalize(ret_img, maxval);

  return err;
}
