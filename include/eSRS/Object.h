// -*- C++ -*-
/** 
 * @file Object.h
 * @brief Class definition for the object to detect or classify
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2019  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __ESRS_OBJECT_H__
#define __ESRS_OBJECT_H__

#include <stdio.h>
#include <vector>
#include <string>
#include "UFV/types.h"

namespace eSRS
{
/*! @struct BoundingBox
 *  @brief 検出結果の矩形描画用構造体
 * 
 *  検出結果の矩形描画用(TODO:正確な用途を確認)
 */
struct BoundingBox : public UFV::Rect
{
  /*!
   *  @brief コンストラクタ
   *  @note       N/A
   *  @attention  N/A
   */
  BoundingBox(void) {};

  /*!
   *  @brief コンストラクタ
   *  @param  [in] ix  基底の構造体(UFV::Rect)に渡す矩形のX座標
   *  @param  [in] iy  基底の構造体(UFV::Rect)に渡す矩形のY座標
   *  @param  [in] w   基底の構造体(UFV::Rect)に渡す矩形のwidth
   *  @param  [in] y   基底の構造体(UFV::Rect)に渡す矩形のheight
   *  @note       N/A
   *  @attention  N/A
   */
  BoundingBox(const int ix, const int iy, const int w, const int h) 
      : UFV::Rect(ix, iy, w, h) {};
};

/*! @struct BoundingVolume
 *  @brief BoundingVolume
 *
 *  
 */
struct BoundingVolume
{

  /*!
   *  @brief コンストラクタ
   *  @note       N/A
   *  @attention  N/A
   */
  BoundingVolume(void) : x(0), y(0), z(0), width(0), height(0), depth(0) {};

  /*!
   *  @brief コンストラクタ
   *  @param  [in] ix  X座標
   *  @param  [in] iy  Y座標
   *  @param  [in] iz  Z座標
   *  @param  [in] w   width
   *  @param  [in] h   height
   *  @param  [in] d   depth
   *  @note       N/A
   *  @attention  N/A
   */
  BoundingVolume(const double ix, const double iy, const double iz,
                 const double w, const double h, const double d) 
      : x(ix), y(iy), z(iz), width(w), height(h), depth(d) {};

  //! X座標
  double x;
  
  //! Y座標
  double y;
  
  //! Z座標
  double z;
  
  //! width
  double width;
  
  //! height
  double height;
  
  //! depth
  double depth;
};

/*! @struct DetectionMap::Obstacle
 *  @brief 検出した物体
 * 
 *  検出した物体の情報
 */
struct Obstacle
{
  //! 
  int             n;
  
  //! 
  BoundingBox     bbox;
  
  //! 
  BoundingVolume  bvol;
  
  //! 
  UFV::Point2D    igrv;

  //! 
  UFV::Point3D    grv;
  
  //! サイズ (pixels)
  int             size;
};

struct ObstacleClassified : public Obstacle
{
  ObstacleClassified(void)
    :
    overlap(0),
    conf(0.0),
    classstr("unknown"),
    color(255.0,0.0,0.0) {};

  ObstacleClassified(const Obstacle &obs)
      : Obstacle(obs),
        overlap(0),
        conf(0.0),
        classstr("unknown"),
        color(255.0,0.0,0.0)
    {
      /*
      this->n = obs.n;
      this->bbox = obs.bbox;
      this->bvol = obs.bvol;
      this->igrv = obs.igrv;
      this->grv = obs.grv;
      this->size = obs.size;
      */
    };
  
  //int n;
  float overlap;
  float conf;
  std::string classstr;
  UFV::Color color;
};

struct YoloObject
{
  int n;

  float conf;

  std::string labelstr;

  UFV::Color color;

  eSRS::BoundingBox bbox;
};

} // namespace eSRS

#endif // __ESRSOBJECT_H__
