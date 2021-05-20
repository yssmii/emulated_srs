// -*- C++ -*-
/**
 * @file types.h
 * @brief Collection of useful definitions
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 * 
 * Copyright (C) 2008  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifndef __LIBESRS_UFV_TYPES_H__
#define __LIBESRS_UFV_TYPES_H__

#ifdef _WIN32
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif
#include <math.h>

namespace UFV
{

enum
{
  OK = 0,
  NG = 1,
  ERR = 2,
//  ERROR = 2,
  END_OF_FILE = -1
};

typedef enum {
  KEY_OK = 0,
  KEY_NG = 1,
  KEY_ERR = 2,
  KEY_QUIT = -1,
  KEY_PAUSE = -2,
  KEY_SAVE = -3
} KeyDef;

typedef enum {
  DEVICE_KINECT_OPENNI2,   //! Kinect + OpenNI2
  DEVICE_KINECT_SDK1,      //! Kinect + SDK 1.8
  DEVICE_KINECT2,          //! Kinect2 + SDK2
  DEVICE_REALSENSE_SR300,   //! RealSense SDK
  DEVICE_REALSENSE_R200,    //! RealSense SDK
  DEVICE_STRUCTURESENSOR,  //! StructureSensor + OpenNI2
  DEVICE_ZED,              //! ZED Stereo Camera + ZED SDK 2.3.x
  DEVICE_XTION2,           //! Xtion2 + OpenNI2
  DEVICE_ASTRA,            //! Orbbec Astra + OpenNI2
  DEVICE_PIXELSOLEIL,      //! PixelSoleil SDK
  DEVICE_REALSENSE_D435,   //! RealSense SDK2
  DEVICE_YVT35,       //! Hokuyo YVT-35LX
  DEVICE_FX10,          //! NipponSignal FX10
  // DEVICE_REALSENSE_D415, //! RealSense SDK
  DEVICE_NUM            //! ストッパー
} Device;

#ifndef _WIN32
typedef unsigned long DWORD;
#endif

const float EpsVal = 1.0e-6;

const double M_1_180 = 1.0 / 180.0;
const double M_180_PI = 180.0 / M_PI;
const double M_PI_180 = M_PI / 180.0;

struct Point2Di // CvPoint2D
{
  Point2Di(void) : x(0), y(0) {};
  Point2Di(const int ix, const int iy) : x(ix), y(iy) {};
  int x;
  int y;
};

struct Point2D // CvPoint2D32f
{
  Point2D(void) : x(0.0), y(0.0) {};
  Point2D(const float ix, const float iy) : x(ix), y(iy) {};
  float x;
  float y;
};

struct Point3D // CvPoint3D32f
{
  Point3D(void) : x(0.0), y(0.0), z(0.0) {};
  Point3D(const float ix, const float iy, const float iz) 
      : x(ix), y(iy), z(iz) {};
  float x;
  float y;
  float z;
};

struct Dimensions2Di  // CvRect で置き換え
{
  Dimensions2Di(void) : width(0), height(0) {};
  Dimensions2Di(const int w, const int h) : width(w), height(h) {};
  int width;   // x-size
  int height;  // y-size
};

struct Dimensions3D
{
  Dimensions3D(void) : width(0.0), height(0.0), depth(0.0) {};
  Dimensions3D(const float w, const float h, const float d)
      : width(w), height(h), depth(d) {};
  float width;   // x-size
  float height;  // y-size
  float depth;   // z-size
};

struct Pose2D
{
  Pose2D(void) : x(0.0), y(0.0), theta(0.0) {};
  Pose2D(const float ix, const float iy, const float it) : 
      x(ix), y(iy), theta(it) {};
  float x;
  float y;
  float theta;
};

struct Quotanion // CvScalar
{
  Quotanion(void) { val[0]=0.0; val[1]=0.0; val[2]=0.0; val[3]=0.0; };
  double val[4];
};

struct Rect // CvRect
{
  Rect(void) : x(0), y(0), width(0), height(0) {};
  Rect(const int ix, const int iy, const int w, const int h) 
  : x(ix), y(iy), width(w), height(h) {};
  int x;
  int y;
  int width;
  int height;
};

struct Color
{
  Color(void) : r(0.0), g(0.0), b(0.0) {};
  Color(const double ir, const double ig, const double ib)
      : r(ir), g(ig), b(ib) {};
  double r;
  double g;
  double b;
};

struct Time
{
  Time(void) : sec(0), usec(0) {};
  Time(const unsigned long s, const unsigned long us) : sec(s), usec(us) {};
  unsigned long sec;        /* seconds */
  unsigned long usec;       /* microseconds */
};

struct LocalTime
{
  LocalTime(void) :
      year(0),
      month(0),
      mday(0),
      hour(0),
      minute(0),
      second(0),
      millisecond(0),
      microsecond(0) {};
  int year;
  int month; // 1 - 12
  int mday;
  int hour;
  int minute;
  int second;
  int millisecond;
  int microsecond;
};

}

#endif
