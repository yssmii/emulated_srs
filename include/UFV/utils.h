// -*- C++ -*-
/**
 * @file utils.h
 * @brief Collection of useful functions
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 * 
 * Copyright (C) 2008  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __LIBESRS_UFV_UTILS_H__
#define __LIBESRS_UFV_UTILS_H__

#include <string>
#include <iostream>
#include <sstream>

#include <stdio.h>

#include "UFV/types.h"

namespace UFV
{

int
kbhit(void);

int
usleep(unsigned int usec);

LocalTime
getLocalTime(void);

LocalTime
getLocalTime(Time &tm);

Time
getTime(void);

double
calcLaptime(Time &from, Time &to);   // returns milliseconds.

class LapTimer
{
public:
  LapTimer() : m_count(0) { m_time = UFV::getTime(); };
  virtual ~LapTimer() {};

  double get(void);

  inline void print(void) { this->print(""); };
  void print(const char *indent="", const char *eol="\n");
  void print(std::ostream &os, const char *indent="", const char *eol="\n");

private:
  long m_count;
  Time m_time;
};

std::string
getLocalTimeString(void);

std::string
getLocalTimeString(LocalTime &lm);


/**
 * Random number generator.
 */
int
getRandomNumber(int rmin, int rmax);

/**
 * Convert radian to degree.
 */
inline double toDeg(const double rad) { return rad * M_180_PI; };

/**
 * Convert degree to radian.
 */
inline double toRad(const double deg) { return deg * M_PI_180; };

/**
 * Convert a 3x3 rotation matrix to X,Y,Z angles (rad).
 *
 * @param R  pointer to double[9]. 3x3 rotation matrix
 * @param[out] X  pointer to double[1].
 * @param[out] Y  pointer to double[1].
 * @param[out] Z  pointer to double[1].
 */
void
convRMatToXYZrad(const double *R, double *X, double *Y, double *Z);

/**
 * Convert a 3x3 rotation matrix to X,Y,Z angles (rad).
 *
 * @param H  pointer to double[12or16]. 3x4 or 4x4 Homogeneous matrix
 * @param[out] X  pointer to double[1].
 * @param[out] Y  pointer to double[1].
 * @param[out] Z  pointer to double[1].
 */
void
convHMatToXYZrad(const double *H, double *X, double *Y, double *Z);

/**
 * Convert X,Y,Z angles to a 3x3 rotation matrix.
 *
 * @param X  rotation angle around X-axis (roll, rad)
 * @param Y  rotation angle around Y-axis (pitch, rad)
 * @param Z  rotation angle around Z-axis (yaw, rad)
 * @param[out] R  pointer to double[9]. 3x3 rotation matrix.
 */
void
convXYZradToRMat(const double X, const double Y, const double Z, double R[9]);

/**
 * Convert an X angle to a 3x3 rotation matrix.
 *
 * @param X  rotation angle around X-axis (roll, rad)
 * @param[out] R  pointer to double[9]. 3x3 rotation matrix.
 */
void
convXradToRMat(const double X, double R[9]);

/**
 * Convert a Y angle to a 3x3 rotation matrix.
 *
 * @param Y  rotation angle around Y-axis (pitch, rad)
 * @param[out] R  pointer to double[9]. 3x3 rotation matrix.
 */
void
convYradToRMat(const double Y, double R[9]);

/**
 * Convert a Z angle to a 3x3 rotation matrix.
 *
 * @param Z  rotation angle around Z-axis (yaw, rad)
 * @param[out] R  pointer to double[9]. 3x3 rotation matrix.
 */
void
convZradToRMat(const double Z, double R[9]);

/**
 * Convert X,Y,Z angles to a 4x4 homogeneous matrix.
 *
 * @param X  rotation angle around X-axis (roll, rad)
 * @param Y  rotation angle around Y-axis (pitch, rad)
 * @param Z  rotation angle around Z-axis (yaw, rad)
 * @param[out] R  pointer to double[12 or 16]. 3x4 or 4x4 homogeneous matrix.
 */
void
convXYZradToHMat(const double X, const double Y, const double Z, double H[12]);

/**
 * Convert an X angle to a 4x4 homogeneous matrix.
 *
 * @param X  rotation angle around X-axis (roll, rad)
 * @param[out] H  pointer to double[12 or 16]. 3x4 or 4x4 homogeneous matrix.
 */
void
convXradToHMat(const double X, double H[12]);

/**
 * Convert a Y angle to a 4x4 homogeneous matrix.
 *
 * @param Y  rotation angle around Y-axis (pitch, rad)
 * @param[out] H  pointer to double[12 or 16]. 3x4 or 4x4 homogeneous matrix.
 */
void
convYradToHMat(const double Y, double H[12]);

/**
 * Convert a Z angle to a 4x4 homogeneous matrix.
 *
 * @param Z  rotation angle around Z-axis (yaw, rad)
 * @param[out] H  pointer to double[12 or 16]. 3x4 or 4x4 homogeneous matrix.
 */
void
convZradToHMat(const double Z, double H[12]);

void
mult3x3Matrix(const double A[9], const double B[9], double C[9]);

void
mult4x4Matrix(const double A[16], const double B[16], double C[16]);

/**
 * Returns a 2D point which is the projection of a 3D point on an image plane.
 * 
 * @param A[9]  intrinsic camera parameters
 * @param R[9]  extrinsic camera parameters (rotation matrix)
 * @param T[3]  extrinsic camera parameters (translation vector)
 */
Point2D
getProjection(const double A[9],
              const double R[9],
              const double T[3],
              const Point3D &poi);

/**
 * Convert a number to std::string.
 */
template <class T>
std::string
numtos(T num)
{
  std::stringstream str_stream;
  str_stream << num;
  return str_stream.str();
}

/**
 * Convert a number to std::string in hex.
 */
template <class T>
std::string
hextos(T num)
{
  std::stringstream str_stream;
  str_stream.setf(std::ios_base::showpos|std::ios_base::hex,
                  std::ios_base::showpos|std::ios_base::basefield);
  str_stream << num;
  return str_stream.str();
}

}  // namespace UFV



#endif
