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
 *  @brief Bounding box representing a detected object in a depth image
 *
 *  - Image coordinate system with origin at upper left
 *  - Unit: pixels
 */
struct BoundingBox : public UFV::Rect
{
  /*!
   *  @brief Constructor
   *  @note       N/A
   *  @attention  N/A
   */
  BoundingBox(void) {};

  /*!
   *  @brief Constructor
   *  @param  [in] ix  X coordinate of the upper left
   *  @param  [in] iy  Y coordinate of the upper left
   *  @param  [in] w   witdth
   *  @param  [in] y   height
   *  @note       N/A
   *  @attention  N/A
   */
  BoundingBox(const int ix, const int iy, const int w, const int h) 
      : UFV::Rect(ix, iy, w, h) {};
};

/*! @struct BoundingVolume
 *  @brief Bounding cuboid representing a detected object in 3D
 *
 *  - Right-handed Sensor-coordinate system
 *  - Unit: mm
 */
struct BoundingVolume
{

  /*!
   *  @brief Constructor
   *  @note       N/A
   *  @attention  N/A
   */
  BoundingVolume(void) : x(0), y(0), z(0), width(0), height(0), depth(0) {};

  /*!
   *  @brief Constructor
   *  @param  [in] ix  X coordinate of the upper left front
   *  @param  [in] iy  Y coordinate of the upper left front
   *  @param  [in] iz  Z coordinate of the upper left front
   *  @param  [in] w   width
   *  @param  [in] h   height
   *  @param  [in] d   depth
   *  @note       N/A
   *  @attention  N/A
   */
  BoundingVolume(const double ix, const double iy, const double iz,
                 const double w, const double h, const double d) 
      : x(ix), y(iy), z(iz), width(w), height(h), depth(d) {};

  double x;
  double y;
  double z;
  double width;
  double height;
  double depth;
};

/*! @struct DetectionMap::Obstacle
 *  @brief Obstacle detected in a depth image
 */
struct Obstacle
{
  //! sequential number of the obstacle
  int             n;
  
  //! 2D location in the image coordinate
  BoundingBox     bbox;
  
  //! 3D location in the sensor coordinate system
  BoundingVolume  bvol;
  
  //! 2D centroid of the obstacle (not center of the bounding box)
  UFV::Point2D    igrv;

  //! 3D centroid of the obstacle (not center of the bounding volume)
  UFV::Point3D    grv;
  
  //! number of the points of the obstacle [pixels]
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
        color(255.0,0.0,0.0) {};
  
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
