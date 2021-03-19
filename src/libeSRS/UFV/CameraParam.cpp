/**
 * Copyright (C) 2008  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#include <string.h>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <fstream>
#include <cctype>
#include <iomanip>

#include "UFV/types.h"
#include "UFV/CameraParam.h"

//#define DEBUG 1

//#define DEBUG2 1

//using namespace UFV;

UFV::CameraParam::CameraParam(void)
    : m_width(0),
      m_height(0)
{
#ifdef DEBUG
  std::cerr << "CameraParam default const" << std::endl;
#endif
#ifdef DEBUG2
  m_intrinsic_matrix = new double[9];
  m_extrinsic_matrix = new double[12];
  m_distortion_coeffs = new double[4];
#endif
  this->_init_param();
}

void
UFV::CameraParam::_init_param(void)
{
  for(int i=0; i<3; ++i)
  {
    for(int j=0; j<3; ++j)
      m_intrinsic_matrix[i*3+j] = (i==j) ? 1.0 : 0.0;

    for(int k=0; k<4; ++k)
      m_extrinsic_matrix[i*4+k] = (i==k) ? 1.0 : 0.0;
  }

  for(int i=0; i<4; ++i)  m_distortion_coeffs[i] = 0.0;
}

UFV::CameraParam::CameraParam(const int width,
                              const int height,
                              const double intrinsic_matrix[9],
                              const double extrinsic_matrix[12],
                              const double distortion_coeffs[4])
    : m_width(width),
      m_height(height)
{
#ifdef DEBUG2
  m_intrinsic_matrix = new double[9];
  m_extrinsic_matrix = new double[12];
  m_distortion_coeffs = new double[4];
#endif

  this->_init_param();

  for(int i=0; i<9; i++)
    m_intrinsic_matrix[i] = intrinsic_matrix[i];
  
  if(extrinsic_matrix != NULL)
  {
    for(int i=0; i<12; i++)
      m_extrinsic_matrix[i] = extrinsic_matrix[i];
  }

  if(distortion_coeffs != NULL)
  {
    for(int i=0; i<4; i++)
      m_distortion_coeffs[i] = distortion_coeffs[i];
  }
}

UFV::CameraParam::CameraParam(std::istream &is, const int nth_yamldoc)
    throw(UFV::CameraParam::BadParamException)
    : m_width(0),
      m_height(0)
{
#ifdef DEBUG2
  m_intrinsic_matrix = new double[9];
  m_extrinsic_matrix = new double[12];
  m_distortion_coeffs = new double[4];
#endif
  this->_init_param();

  this->_load_param(is, &m_width, &m_height,
                    m_intrinsic_matrix, m_extrinsic_matrix,
                    m_distortion_coeffs, nth_yamldoc);
}

UFV::CameraParam::CameraParam(const char *fpath, const int nth_yamldoc)
    throw(UFV::CameraParam::IOError,
          UFV::CameraParam::BadParamException)
    : m_width(0),
      m_height(0)
{
#ifdef DEBUG2
  m_intrinsic_matrix = new double[9];
  m_extrinsic_matrix = new double[12];
  m_distortion_coeffs = new double[4];
#endif

  this->_init_param();

  std::ifstream ifs(fpath);
  if(ifs)
  {
    this->_load_param(ifs, &m_width, &m_height,
                      m_intrinsic_matrix, m_extrinsic_matrix,
                      m_distortion_coeffs, nth_yamldoc);
  }
  else
  {
    std::string msg("CANNOT OPEN: ");
    throw UFV::CameraParam::IOError((msg+fpath).c_str());
  }
  ifs.close();
}

UFV::CameraParam::~CameraParam(void)
{
#ifdef DEBUG2
  delete [] m_intrinsic_matrix;
  delete [] m_extrinsic_matrix;
  delete [] m_distortion_coeffs;
#endif
}

// Copy Constructor
UFV::CameraParam::CameraParam(const CameraParam &cp)
    : m_width(cp.getWidth()),
      m_height(cp.getHeight())
{
#ifdef DEBUG2
  m_intrinsic_matrix = new double[9];
  m_extrinsic_matrix = new double[12];
  m_distortion_coeffs = new double[4];
#endif

#ifdef DEBUG
  std::cerr << "CameraParam copy" << std::endl;
#endif

  cp.getIntrinsicMatrix(m_intrinsic_matrix);
  cp.getExtrinsicMatrix(m_extrinsic_matrix);
  cp.getDistortionCoeffs(m_distortion_coeffs);
}

UFV::CameraParam &
UFV::CameraParam::operator=(const UFV::CameraParam &cp)
{
  if(this == &cp) return *this;

#ifdef DEBUG
  std::cerr << "CameraParam operator=" << std::endl;
#endif

  m_width = cp.getWidth();
  m_height = cp.getHeight();

  cp.getIntrinsicMatrix(m_intrinsic_matrix);
  cp.getExtrinsicMatrix(m_extrinsic_matrix);
  cp.getDistortionCoeffs(m_distortion_coeffs);

  return *this;
}

void
UFV::CameraParam::setExtrinsicMatrix(const double extrinsic_matrix[12])
{
  for(int i=0; i<12; i++)
    m_extrinsic_matrix[i] = extrinsic_matrix[i];
  return;
}

void
UFV::CameraParam::setIntrinsicMatrix(const double intrinsic_matrix[9])
{
  for(int i=0; i<9; i++)
    m_intrinsic_matrix[i] = intrinsic_matrix[i];
  return;
}

void
UFV::CameraParam::setDistortionCoeffs(const double coeffs[4])
{
  m_distortion_coeffs[0] = coeffs[0];
  m_distortion_coeffs[1] = coeffs[1];
  m_distortion_coeffs[2] = coeffs[2];
  m_distortion_coeffs[3] = coeffs[3];
  return;
}

void
UFV::CameraParam::getDistortionCoeffs(double coeffs[4]) const 
{
  coeffs[0] = m_distortion_coeffs[0];
  coeffs[1] = m_distortion_coeffs[1];
  coeffs[2] = m_distortion_coeffs[2];
  coeffs[3] = m_distortion_coeffs[3];
  return;
}

void
UFV::CameraParam::getIntrinsicMatrix(double intrinsic_matrix[9]) const
{
  for(int i=0; i<9; i++)
    intrinsic_matrix[i] = m_intrinsic_matrix[i];
  return;
}

void
UFV::CameraParam::getExtrinsicMatrix(double extrinsic_matrix[12]) const
{
  for(int i=0; i<12; i++)
    extrinsic_matrix[i] = m_extrinsic_matrix[i];
  return;
}

void
UFV::CameraParam::getExtrinsicMatrix(double rotation_matrix[9],
                                     double translation_vector[3]) const
{
  for(int i=0; i<3; i++)
  {
    rotation_matrix[i*4] = m_extrinsic_matrix[i*4];
    rotation_matrix[i*4+1] = m_extrinsic_matrix[i*4+1];
    rotation_matrix[i*4+2] = m_extrinsic_matrix[i*4+2];
    translation_vector[i] = m_extrinsic_matrix[i*4+3];
  }
  return;
}

/*********************************************************************
 * Read camera intrinsic parameters
 */
static bool
is_space_line(std::string &buf)
{
  for(unsigned int i=0; i<buf.size(); ++i)
  {
    if(!std::isspace(buf[i]))
      return false;
  }
  
  return true;    /* すべての文字がSPACE    */
}

static bool
is_comment_line(std::string &buf)
{
  if(buf[0] == '#')
    return true;
  return false;
}

static int
read_nextline(std::istream &is, std::string &buf)
{
  while(1)
  {
    if(std::getline(is, buf))
    {
      if(is_space_line(buf) || is_comment_line(buf))
        continue;
      else
        return(UFV::OK);
    }
    else
      return(UFV::END_OF_FILE);
  }
}

static void
read_sizedata(int *width, int *height, std::istream &is, std::string &buf)
{
  int ret = read_nextline(is, buf);
  if(ret != UFV::OK)
  {
    if(ret == UFV::END_OF_FILE)
      throw UFV::CameraParam::BadParamException("BAD SIZE PARAMS");
  }

  std::istringstream iss(buf);  // " [ %d, %d ]"
  char c;

  iss >> c >> *width;     // [ %d
  iss >> c >> *height;    // , %d

  if(iss.fail() == true)
    throw UFV::CameraParam::BadParamException("BAD SIZE PARAMS");

  return;
}

static void
read_extrinsicdata(double *E, std::istream &is, std::string &buf)
{
  for(int i=0; i<3; i++)
  {
    int ret = read_nextline(is, buf);
    if(ret != UFV::OK)
    {
      if(ret == UFV::END_OF_FILE)
        throw UFV::CameraParam::BadParamException("BAD EXTRINSIC PARAMS");
    }

    std::istringstream iss(buf);  // " - [ %lf, %lf, %lf, %lf ]"
    char c;

    iss >> c;              // -
    iss >> c >> E[i*4];    // [ %lf
    iss >> c >> E[i*4+1];  // , %lf
    iss >> c >> E[i*4+2];  // , %lf
    iss >> c >> E[i*4+3];  // , %lf

    if(iss.fail() == true)
      throw UFV::CameraParam::BadParamException("BAD EXTRINSIC PARAMS");
  }

  return;
}

static void
read_intrinsicdata(double *I, std::istream &is, std::string &buf)
{
  for(int i=0; i<3; i++)
  {
    int ret = read_nextline(is, buf);
    if(ret != UFV::OK)
    {
      if(ret == UFV::END_OF_FILE)
        throw UFV::CameraParam::BadParamException("BAD INRINSIC PARAMS");
    }

    std::istringstream iss(buf);  // " - [ %lf, %lf, %lf ]"
    char c;

    iss >> c;              // -
    iss >> c >> I[i*3];    // [ %lf
    iss >> c >> I[i*3+1];  // , %lf
    iss >> c >> I[i*3+2];  // , %lf

    if(iss.fail() == true)
      throw UFV::CameraParam::BadParamException("BAD INRINSIC PARAMS");
  }

  return;
}


static void
read_distortiondata(double *D, std::istream &is, std::string &buf)
{
  int ret = read_nextline(is, buf);
  if(ret != UFV::OK)
  {
    if(ret == UFV::END_OF_FILE)
      throw UFV::CameraParam::BadParamException("BAD DISTORTION PARAMS");
  }

  std::istringstream iss(buf);  // " [ %lf, %lf, %lf, %lf ]"
  char c;

  iss >> c >> D[0];  // [ %lf
  iss >> c >> D[1];  // , %lf
  iss >> c >> D[2];  // , %lf
  iss >> c >> D[3];  // , %lf

  if(iss.fail() == true)
    throw UFV::CameraParam::BadParamException("BAD DISTORTION PARAMS");
  
  return;
}

static bool
is_separator_line(std::string &buf)
{
  if( buf.compare(0, 3, "---") )
    return false;
  else
    return true;
}

int
UFV::CameraParam::_load_param(std::istream &is, 
                              int *width, int *height,
                              double *intr, double *extr,
                              double *dstr,
                              const int nth_doc)
{
  int flagsize=0;         /* 必須情報     */
  int flagintrinsic=0;    /* 必須情報     */
  int is_multiyaml=0;
  int ret = UFV::OK;

  int count=nth_doc;

  std::string buf;

  while(1)
  {
    ret = read_nextline(is, buf);
    if(ret != UFV::OK)
    {
      if(ret == UFV::END_OF_FILE)
        throw UFV::CameraParam::BadParamException("BAD FORMAT");
    }

    if(is_separator_line(buf))
    {
      is_multiyaml = 1;
      count--;
    }
    else
    {
      if(!(is_multiyaml && count >= 0))
        break;
    }
  }

  while(1)
  {
    if(buf.compare(0, strlen("SIZE:"), "SIZE:") == 0)
    {
      read_sizedata(width, height, is, buf);
      flagsize = 1;
    }
    else if(buf.compare(0, strlen("INTRINSIC:"), "INTRINSIC:") == 0)
    {
      read_intrinsicdata(intr, is, buf);
      flagintrinsic = 1;
    }
    else if(buf.compare(0, strlen("EXTRINSIC:"), "EXTRINSIC:") == 0)
    {
      read_extrinsicdata(extr, is, buf);
    }
    else if(buf.compare(0, strlen("DISTORTION:"), "DISTORTION:") == 0)
    {
      read_distortiondata(dstr, is, buf);
    }
    
    ret = read_nextline(is, buf);
    if(ret == UFV::END_OF_FILE || is_separator_line(buf))
    {
      if(!(flagsize && flagintrinsic))
        throw UFV::CameraParam::BadParamException("NO SIZE and/or INTRINSIC PARAMS.");
      break;
    }
  }

#ifdef DEBUG
  this->print(std::cerr);
#endif  
  
  return(UFV::OK);
}

/*********************************************************************
 * Print Checker Pattern
 */
int
UFV::CameraParam::print(std::ostream &os, const char *indent) const
{
  os << indent << "SIZE:" << std::endl;
  os << indent << "  [ " << m_width << ", " << m_height << " ]" << std::endl;

  os << indent << "DISTORTION:" << std::endl;
  os << indent 
     << "  [ " 
     << std::setw(10) << std::right << m_distortion_coeffs[0] << ", "
     << std::setw(10) << std::right << m_distortion_coeffs[1] << ", "
     << std::setw(10) << std::right << m_distortion_coeffs[2] << ", "
     << std::setw(10) << std::right << m_distortion_coeffs[3] << " ]" 
     << std::endl;

  os << indent << "INTRINSIC:" << std::endl;
  for(int i=0; i<3; i++)
    os << indent 
       << "  - [ " 
       << std::setw(10) << std::right << m_intrinsic_matrix[i*3]   << ", "
       << std::setw(10) << std::right << m_intrinsic_matrix[i*3+1] << ", "
       << std::setw(10) << std::right << m_intrinsic_matrix[i*3+2] << " ]"
       << std::endl;

  os << indent << "EXTRINSIC:" << std::endl;
  for(int i=0; i<3; i++)
    os << indent 
       << "  - [ " 
       << std::setw(10) << std::right << m_extrinsic_matrix[i*4]   << ", "
       << std::setw(10) << std::right << m_extrinsic_matrix[i*4+1] << ", "
       << std::setw(10) << std::right << m_extrinsic_matrix[i*4+2] << ", "
       << std::setw(10) << std::right << m_extrinsic_matrix[i*4+3] << " ]" 
       << std::endl;

  return UFV::OK;
}

