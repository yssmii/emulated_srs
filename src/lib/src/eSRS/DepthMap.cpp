/*!
 * @file  DepthMap.cpp
 * @brief DepthMap class implementation
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2013  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include <float.h>
#include <string>
#include <opencv2/opencv.hpp> 
#include <opencv2/highgui/highgui.hpp> 

#include <iostream>

#include "UFV/types.h"
#include "UFV/keydefs.h"
#include "UFV/ImageData.h"

#include "eSRS/DepthMap.h"

const double eSRS::DepthMap::DEFAULT_MAXDEPTH = 10000.0;    // mm

//const int eSRS::DepthMap::COLOR_NODATA[] = {0, 255, 255};
const int eSRS::DepthMap::COLOR_NODATA[] = {0, 0, 0};
//const int eSRS::DepthMap::COLOR_SATURATION[] = {255, 0, 255};
const int eSRS::DepthMap::COLOR_SATURATION[] = {0, 0, 0};
const int eSRS::DepthMap::COLOR_UNKNOWN[] = {255, 0, 0};
//const int eSRS::DepthMap::COLOR_MASKED[] = {139, 0, 0};
const int eSRS::DepthMap::COLOR_MASKED[] = {0, 0, 0};
const int eSRS::DepthMap::COLOR_OTHER[] = {0, 0, 255};
const int eSRS::DepthMap::COLOR_OUTOFRANGE[] = {0, 0, 0};

/**
 * 0~254 : depth data
 * 255 : invalid data / no data
 */
static void
_normalize(const eSRS::DepthMap &srcmap, 
           UFV::ImageData<unsigned char> &dstimg,
           double md = -1.0)
{
  float maxdepth = (md < 0.0) ? srcmap.getMaxDepth() : md; // mm
  int width = srcmap.width();
  int height = srcmap.height();

  float *pSrcMap = srcmap.data();
  unsigned char *pDstImg = dstimg.data();

  double scale = 255.0 / maxdepth;
  for(int i=0; i<width*height; ++i)
  {
    if(*pSrcMap < -FLT_EPSILON) // 負値
      *pDstImg = 255;
    else
    {
      double tmpdata = scale * (*pSrcMap);
      double tmpval = 255 - tmpdata;
      if(tmpval < -FLT_EPSILON)
      {
        *pDstImg = 255;
      }
      else if(fabs(tmpval) < FLT_EPSILON)
        *pDstImg = 0;
      else
        *pDstImg = tmpval;
    }
    ++pDstImg;
    ++pSrcMap;
  }
  
  return;
}

static void
_normalizeBGR(const eSRS::DepthMap &srcmap, 
              UFV::ImageData<unsigned char> &dstimg,
              double md = -1.0)
{
  float maxdepth = (md < 0.0) ? srcmap.getMaxDepth() : md; // mm
  int width = srcmap.width();
  int height = srcmap.height();

  float *pSrcMap = srcmap.data();
  unsigned char *pDstImg = dstimg.data();

  double scale = 255.0 / maxdepth;
  for(int i=0, j=0; i<width*height; ++i, j+=3)
  {
    if(fabs(*pSrcMap - eSRS::DepthMap::PIXELVALUE_NODATA) 
       < FLT_EPSILON) // Yellow
    {
      *pDstImg++ = eSRS::DepthMap::COLOR_NODATA[0];   // B
      *pDstImg++ = eSRS::DepthMap::COLOR_NODATA[1]; // G
      *pDstImg++ = eSRS::DepthMap::COLOR_NODATA[2]; // R
    }
    else if(fabs(*pSrcMap - eSRS::DepthMap::PIXELVALUE_SATURATION)
            < FLT_EPSILON) // MAGENTA
    {
      *pDstImg++ = eSRS::DepthMap::COLOR_SATURATION[0];   // B
      *pDstImg++ = eSRS::DepthMap::COLOR_SATURATION[1]; // G
      *pDstImg++ = eSRS::DepthMap::COLOR_SATURATION[2]; // R
    }
    else if(fabs(*pSrcMap - eSRS::DepthMap::PIXELVALUE_UNKNOWN)
            < FLT_EPSILON) // BLUE
    {
      *pDstImg++ = eSRS::DepthMap::COLOR_UNKNOWN[0];   // B
      *pDstImg++ = eSRS::DepthMap::COLOR_UNKNOWN[1]; // G
      *pDstImg++ = eSRS::DepthMap::COLOR_UNKNOWN[2]; // R
    }
    else if(fabs(*pSrcMap - eSRS::DepthMap::PIXELVALUE_MASKED)
            < FLT_EPSILON) // BLUE
    {
      *pDstImg++ = eSRS::DepthMap::COLOR_MASKED[0];   // B
      *pDstImg++ = eSRS::DepthMap::COLOR_MASKED[1]; // G
      *pDstImg++ = eSRS::DepthMap::COLOR_MASKED[2]; // R
    }
    else if(*pSrcMap < -1.0)  // それ以外の負値。アプリ依存: RED
    {
      *pDstImg++ = eSRS::DepthMap::COLOR_OTHER[0];   // B
      *pDstImg++ = eSRS::DepthMap::COLOR_OTHER[1]; // G
      *pDstImg++ = eSRS::DepthMap::COLOR_OTHER[2]; // R
    }
    else // ゼロまたは正値
    {
      double tmpdata = 255 - scale * (*pSrcMap);
      if( tmpdata < -FLT_EPSILON ) // 範囲外 CYAN
      {
        *pDstImg++ = eSRS::DepthMap::COLOR_OUTOFRANGE[0];   // B
        *pDstImg++ = eSRS::DepthMap::COLOR_OUTOFRANGE[1];   // G
        *pDstImg++ = eSRS::DepthMap::COLOR_OUTOFRANGE[2];   // R
        //*pDstImg++ = tmpdata;
        //*pDstImg++ = tmpdata;
        //*pDstImg++ = 0;
      }
      else if(fabs(tmpdata) < FLT_EPSILON )
      {
        *pDstImg++ = 0;
        *pDstImg++ = 0;
        *pDstImg++ = 0;
      }
      else
      {
        *pDstImg++ = tmpdata;
        *pDstImg++ = tmpdata;
        *pDstImg++ = tmpdata;
      }
    }
    pSrcMap++;
  }

  return;
}

int
eSRS::DepthMap::normalize(UFV::ImageData<unsigned char> &img,
                          double maxdepth) const
{
  if(this->width() != img.width() || this->height() != img.height())
    return UFV::NG;

  if(img.nchannels() == 3)
  {
    _normalizeBGR(*this, img, maxdepth);
  }
  else if(img.nchannels() == 1)
  {
    _normalize(*this, img, maxdepth);
  }
  else
    return UFV::NG;

  return UFV::OK;
}

int
eSRS::DepthMap::display(const std::string wlabel, const int msec,
                        double maxdepth) const
{
  UFV::ImageData<unsigned char> showimg(this->width(), this->height(), 3);
  int ret = this->normalize(showimg,maxdepth);
  if(ret != UFV::OK) return ret;

  cv::Mat simg(cv::Size(this->width(), this->height()), CV_8UC3, 
               showimg.data());
  cv::imshow(wlabel, simg);
  if(msec >= 0)
  {
    int rkey = cv::waitKey(msec);
    if(rkey == XK_q)
      return UFV::END_OF_FILE;
  }

  return UFV::OK;
}

int
eSRS::DepthMap::recoverDepth(const UFV::ImageData<unsigned char> &img)
{
  int width = this->width();
  int height = this->height();

  if(img.width() != width || img.height() != height ||
     img.nchannels() != 3)
    return UFV::NG;

  double scale = this->getMaxDepth() / 255.0;

  float *pMap = this->data();
  unsigned char *pImg = img.data();

  for(int i=0, j=0; i<width*height; ++i, j+=3)
  {
    if(pImg[j] == eSRS::DepthMap::COLOR_NODATA[0] &&       // B
       pImg[j+1] == eSRS::DepthMap::COLOR_NODATA[1] &&   // G
       pImg[j+2] == eSRS::DepthMap::COLOR_NODATA[2])     // R : Yellow
    {
      pMap[i] = eSRS::DepthMap::PIXELVALUE_NODATA;
    }
    else if(pImg[j] == eSRS::DepthMap::COLOR_SATURATION[0] &&   // B
            pImg[j+1] == eSRS::DepthMap::COLOR_SATURATION[1] && // G
            pImg[j+2] == eSRS::DepthMap::COLOR_SATURATION[2] )  // R : MAGENTA
    {
      pMap[i] = eSRS::DepthMap::PIXELVALUE_SATURATION;
    }
    else if(pImg[j] == eSRS::DepthMap::COLOR_UNKNOWN[0]  &&       // B
            pImg[j+1] == eSRS::DepthMap::COLOR_UNKNOWN[1] &&   // G
            pImg[j+2] == eSRS::DepthMap::COLOR_UNKNOWN[2])     // R : BLUE
    {
      pMap[i] = eSRS::DepthMap::PIXELVALUE_UNKNOWN;
    }
    else if(pImg[j] == eSRS::DepthMap::COLOR_MASKED[0] &&       // B
            pImg[j+1] == eSRS::DepthMap::COLOR_MASKED[1] &&   // G
            pImg[j+2] == eSRS::DepthMap::COLOR_MASKED[2])     // R : BLUE
    {
      pMap[i] = eSRS::DepthMap::PIXELVALUE_MASKED;
    }
    else if(pImg[j] == eSRS::DepthMap::COLOR_OUTOFRANGE[0] &&       // B
            pImg[j+1] == eSRS::DepthMap::COLOR_OUTOFRANGE[1] &&   // G
            pImg[j+2] == eSRS::DepthMap::COLOR_OUTOFRANGE[0])     // R : RED
    {
      pMap[i] = eSRS::DepthMap::PIXELVALUE_SATURATION; // 正しいか?
    }
    else if(pImg[j] == pImg[j+1] &&
            pImg[j] == pImg[j+2])   // GRAY
    {
      pMap[i] = (255 - pImg[j]) * scale;
    }
    else
    {
      pMap[i] = eSRS::DepthMap::PIXELVALUE_UNKNOWN;
    }
  }

  return UFV::OK;
}

void
eSRS::DepthMap::resetMaskImage(void)
{
  UFV::ImageData<unsigned char> m;
  m_mask = m;

  return;
}

int
eSRS::DepthMap::setMaskImage(const UFV::ImageData<unsigned char> &mask)
{
  if(mask.width() != this->width() || mask.height() != this->height() ||
     mask.nchannels() != 1)
    return UFV::NG;

  m_mask = mask;

  return UFV::OK;
}

int
eSRS::DepthMap::setMaskImage(const std::string &maskfile)
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
eSRS::DepthMap::mask(void)
{
  if(!this->hasMaskImage()) return UFV::NG;

  float *pdat = this->data();
  unsigned char *pmask = m_mask.data();

  for(int i=0; i<this->width()*this->height(); i++)
  {
    if(*pmask == 0)
      *pdat = PIXELVALUE_MASKED;
    pmask++;
    pdat++;
  }
  
  return UFV::OK;
}
