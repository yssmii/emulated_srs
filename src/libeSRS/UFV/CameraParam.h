// -*- C++ -*-
/** 
 * @file CameraParam.h
 * @brief UFV::CameraParam class definition
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2008  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __LIBESRS_UFV_CAMERAPARAM_H__
#define __LIBESRS_UFV_CAMERAPARAM_H__

#include <iostream>
#include <string>
#include <stdexcept>

namespace UFV 
{

class CameraParam
{
public:
  class IOError : public std::ios_base::failure
  {
  public:
    IOError(const std::string &msg) : std::ios_base::failure(msg) {};
  };
  class BadParamException : public std::domain_error
  {
  public:
    BadParamException(const std::string &msg) : std::domain_error(msg) {};
  };

public:
  CameraParam(void);
  CameraParam(const char *path, const int nth_yamldoc=0) 
    throw(IOError, BadParamException);
  CameraParam(std::istream &is, const int nth_yamldoc=0)
    throw(BadParamException);
  CameraParam(const int width, 
              const int height, 
              const double intrinsic_matrix[9],
              const double extrinsic_matrix[12],
              const double distortion_coeffs[4]);
  virtual ~CameraParam(void);

  CameraParam(const CameraParam &cp);
  CameraParam& operator=(const CameraParam &cp);

  int print(std::ostream &os, const char *indent="") const;

  int width(void) const {return m_width; };
  int height(void) const { return m_height; };

  void setWidth(const int width) { m_width = width; };
  void setHeight(const int height) { m_height = height; };
  int getWidth(void) const {return m_width; };
  int getHeight(void) const { return m_height; };

  void setDistortionCoeffs(const double coeffs[4]);
  void setIntrinsicMatrix(const double intrinsic_matrix[9]);
  void setExtrinsicMatrix(const double extrinsic_matrix[12]);

  void getDistortionCoeffs(double coeffs[4]) const;
  void getIntrinsicMatrix(double intrinsic_matrix[9]) const;
  void getExtrinsicMatrix(double extrinsic_matrix[12]) const; 
  void getExtrinsicMatrix(double rotation_matrix[9],
                          double translation_vector[3]) const;

private:
  int _load_param(std::istream &is,
                  int *width, int *height,
                  double *intr, double *extr, 
                  double *dstr,
                  const int nth_yamldoc);

  void _init_param(void);

private:
  int m_width;
  int m_height;
  double m_intrinsic_matrix[9];
  double m_extrinsic_matrix[12];
  double m_distortion_coeffs[4];
};

} // namespace UFV

#endif
