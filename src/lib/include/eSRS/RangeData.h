// -*- C++ -*-
/** 
 * @file RangeData.h
 * @brief bas
 *
 * @author Yasushi SUMI (AIST)
 *
 * Copyright (C) 2013  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __LIBESRS_ESRS_RANGEDATA_H__
#define __LIBESRS_ESRS_RANGEDATA_H__

#include <iostream>
#include <string>
#include <stdexcept>

#include "UFV/RangeData.h"

#include "eSRS/DepthMap.h"

namespace eSRS
{

class RangeData : public UFV::RangeData
{
public:
  RangeData(const char *path, const int maxdepth) :
      UFV::RangeData(path, maxdepth) {};

  RangeData(std::istream &is, const int maxdepth) :
      UFV::RangeData(is, maxdepth) {};

  RangeData(const int width, const int height, const int maxdepth) :
      UFV::RangeData(width, height, maxdepth) {};

  virtual ~RangeData(void) {};

public:
  virtual void getMaps(eSRS::DepthMap &depthmap) const;
};

} // namespace UFV

#endif
