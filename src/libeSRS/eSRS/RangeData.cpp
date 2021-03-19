/**
 * Copyright (C) 2013  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include <float.h>

#include "UFV/types.h"
#include "UFV/IntensityMap.h"
#include "eSRS/RangeData.h"
#include "eSRS/DepthMap.h"

//#define DEBUGI 1

void
eSRS::RangeData::getMaps(eSRS::DepthMap &depthmap) const
{
  int width = this->getWidth();
  int height = this->getHeight();
  //const int SATULATEDINTENSITY=255;

  if(depthmap.width() != width || depthmap.height() != height)
    depthmap.reshape(width, height);     // サイズをRangeDataに合わせて、

  double maxz = this->getSensorMaxDepth();
  depthmap.setMaxDepth(maxz);            // RangeData最大距離をセットして、

  const double *pData = this->getData(); // RangeDataのxyzデータのうち、

  //UFV::ImageData<float> *xmap = depthmap.getXmap(); // x値格納用
  //UFV::ImageData<float> *ymap = depthmap.getYmap(); // y値格納用

  float *pDepthmap = depthmap.data();    // 内部の2Dデータに、
  float *pXmap = depthmap.xdata();      // x値格納用
  float *pYmap = depthmap.ydata();      // y値格納用
  
  for(int i=0, j=0; i<width*height; i++, j+=3) // 値をコピーする。
  {
    pXmap[i] = pData[j];         // X
    pYmap[i] = pData[j+1];       // Y
    double zdata = pData[j+2];   // Z          
    if(zdata < 0)                          // Zが負なら、TOOFARかNODATA
    {
      if(fabs(zdata - ZVAL_TOOFAR) < FLT_EPSILON)
        pDepthmap[i] = eSRS::DepthMap::PIXELVALUE_SATULATION;
      else
        pDepthmap[i] = eSRS::DepthMap::PIXELVALUE_NODATA;
    }
    else if(zdata > maxz)                // 最大距離より大きければTOOFAR
      pDepthmap[i] = eSRS::DepthMap::PIXELVALUE_SATULATION;
    else
      pDepthmap[i] = zdata;
  }

  return;
}
