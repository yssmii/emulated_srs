// -*- C++ -*-
/*!
 * @file  ClassificationMap.h
 * @brief definition of DetectionMap extention for Yolo
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2019  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifndef __LIBESRS_ESRS_CLASSIFICATIONMAP_H__
#define __LIBESRS_ESRS_CLASSIFICATIONMAP_H__

#include "eSRS/Object.h"
#include "eSRS/DetectionMap.h"

namespace eSRS
{

class ClassificationMap : public eSRS::DetectionMap
{
public:
  static const double DEFAULT_MIN_OVERLAP_RATE;

public:
  ClassificationMap(void) {};
  ClassificationMap(const int width, const int height, const float *data=0,
                    const double zkey=DEFAULT_ZKEY,
                    const double mingap=DEFAULT_MIN_GAP,
                    const int minsize=DEFAULT_MIN_SIZE,
                    const double minoverlap=DEFAULT_MIN_OVERLAP_RATE) :
      eSRS::DetectionMap(width, height, data, zkey, mingap, minsize) {};
  virtual ~ClassificationMap(void) {};

public:
  virtual void reshape(const int width, const int height) {
    this->eSRS::DetectionMap::reshape(width, height);
    m_ocls_vec.clear();
    return;
  };

public:
  int detect(void); //! YOLOとの統合なし。分類はすべて unknown になる
  int detect(std::vector<YoloObject> &yobjs); //! YOLOの認識結果と統合

   //! YOLOの認識結果と統合。異なるzkeyで二重チェック (experimental)
  int detect(std::vector<YoloObject> &yobjs, const bool flg_doublecheck);

public:
  void getObstacleClassified(std::vector<ObstacleClassified> &ocls) const
    {
      ocls = m_ocls_vec;
    };

public:
  void setMinOverlapRate(const double val) {
    if(val >= 1.0) m_min_overlap_rate = 1.0;
    else if(val <= 0.0) m_min_overlap_rate = 0.0;
    else m_min_overlap_rate = val;
  };
  
public:
  int drawBoundingBox(UFV::ImageData<unsigned char> &bbox) const;
  int drawObstacleRegion(UFV::ImageData<unsigned char> &bbox) const;

protected:
  std::vector<ObstacleClassified> m_ocls_vec;

private:
  float m_min_overlap_rate;
};


} // namespace eSRS


#endif // __ESRS_CLASSIFICATIONMAP_H__



