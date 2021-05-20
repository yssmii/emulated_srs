// -*- C++ -*-
/** 
 * @file DepthMap.h
 * @brief bas
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2013  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __LIBESRS_ESRS_DEPTHMAP_H__
#define __LIBESRS_ESRS_DEPTHMAP_H__

#include <string>

#include "UFV/types.h"
#include "UFV/ImageData.h"

namespace eSRS
{

class DepthMap : public UFV::ImageData<float>
{
public:
  static const int DEFAULT_WIDTH = 640;
  static const int DEFAULT_HEIGHT = 480;
  static const double DEFAULT_MAXDEPTH;      // 10000 mm
  
  static const int PIXELVALUE_NODATA = -1;
  static const int PIXELVALUE_SATURATION = -2;
  static const int PIXELVALUE_UNKNOWN = -3;
  static const int PIXELVALUE_MASKED = -4;

public:
  //static const int COLOR_NODATA[3];     // Yellow
  static const int COLOR_NODATA[3];     // Black
  static const int COLOR_SATURATION[3]; // Magenta
  static const int COLOR_UNKNOWN[3];    // Blue
  static const int COLOR_MASKED[3];     // Dark Blue
  static const int COLOR_OTHER[3];      // Red
  static const int COLOR_OUTOFRANGE[3]; // Cyan

public:
  DepthMap(void) :
      m_maxdepth(DEFAULT_MAXDEPTH) {};
//      UFV::ImageData<float>() {};

  DepthMap(const int width, //=DEFAULT_WIDTH,
           const int height, //=DEFAULT_HEIGHT,
           const float *data=0,
           const double maxdepth=DEFAULT_MAXDEPTH) :
      UFV::ImageData<float>(width, height, 1, data),
      m_maxdepth(maxdepth),
      m_xmap(width, height, 1),
      m_ymap(width, height, 1)
    {};

  template<class T>
  DepthMap(const UFV::ImageData<T> &imgData, const double maxdepth) :
      UFV::ImageData<float>(imgData),
      m_maxdepth(maxdepth),
      m_xmap(imgData),
      m_ymap(imgData)
    {};

  virtual ~DepthMap(void) {};

public:
  virtual int normalize(UFV::ImageData<unsigned char> &img,
                        double maxdepth=-1.0) const;
  virtual UFV::KeyDef display(const std::string wlabel,
                              const int msec,
                              double maxdepth=-1.0) const;

  virtual int recoverDepth(const UFV::ImageData<unsigned char> &img);

  virtual void reshape(const int width, const int height) {
    this->UFV::ImageData<float>::reshape(width, height, 1);
    //this->setData(width, height, 1);
    m_xmap.reshape(width, height, 1);
    m_ymap.reshape(width, height, 1);
    this->resetMaskImage();
    return;
  };


public:
  void setMaxDepth(const double val) { m_maxdepth = val; };
  double getMaxDepth(void) const { return m_maxdepth; };

  void setData(const float *data) {
    UFV::ImageData<float>::setData(this->width(), this->height(), 1, data);
  };
  void setData(const float *data, const double val) {
    UFV::ImageData<float>::setData(this->width(), this->height(), 1, data);
    m_maxdepth = val;
  };

public:
  float *getXData(void) const { return m_xmap.getData(); };
  float *getYData(void) const { return m_ymap.getData(); };

  float *xdata(void) const { return this->getXData(); };
  float *ydata(void) const { return this->getYData(); };

public:
  bool hasMaskImage(void) const { return !m_mask.isEmpty(); };
  int setMaskImage(const UFV::ImageData<unsigned char> &mask);
  int setMaskImage(const std::string &pathtofile);
  void resetMaskImage(void);

  int mask(void);
  template<typename U>
  int mask(UFV::ImageData<U> &img,
           const U val=eSRS::DepthMap::COLOR_MASKED[0],
           const U valg=eSRS::DepthMap::COLOR_MASKED[1],
           const U valr=eSRS::DepthMap::COLOR_MASKED[2]) const;

private:
  double m_maxdepth;             // mm

protected:
  UFV::ImageData<float> m_xmap;
  UFV::ImageData<float> m_ymap;

protected:
  UFV::ImageData<unsigned char> m_mask;
};

template<typename U>
int
eSRS::DepthMap::mask(UFV::ImageData<U> &img,
                     const U val, const U valg, const U valr) const
{
  if(!this->hasMaskImage())
    return UFV::NG;
  if(img.width() != this->width() || img.height() != this->height())
    return UFV::NG;

  U *pdat = img.data();
  unsigned char *pmask = m_mask.data();

  if(img.nchannels() == 1)
  {
    for(int i=0; i<this->width()*this->height(); i++)
    {
      if(*pmask == 0)
        *pdat = val;
      pmask++;
      pdat++;
    }
  }
  else if(img.nchannels() == 3)
  {
    for(int i=0; i<this->width()*this->height(); i++)
    {
      if(*pmask == 0)
      {
        *pdat++ = val;
        *pdat++ = valg;
        *pdat++ = valr;
      }
      else
        pdat += 3;
      pmask++;
    }
  }
  
  return UFV::OK;
}


} // namespace

#endif
