// -*- C++ -*-
/** 
 * @file RangeData.cpp
 * @brief
 *
 * @author Yasushi SUMI (AIST)
 * 
 * Copyright (C) 2011  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#include <string.h>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <fstream>
#include <cctype>

#include "UFV/types.h"
#include "UFV/RangeData.h"

//#define DEBUG 0

using namespace UFV;

// 計測
const double UFV::RangeData::DEFAULT_MAXDEPTH = 10000.0; // mm

RangeData::RangeData(const int width, const int height, const int maxdepth)
    : 
      m_width(width),
      m_height(height),
      m_depthmap(NULL),
      m_maxdepth(maxdepth)
{
  m_data = new double[m_width*m_height*3];
}

RangeData::RangeData(std::istream &is, const int maxdepth)
  throw(BadDataException)
    : 
      m_width(0),
      m_height(0),
      m_data(NULL),
      m_depthmap(NULL),
      m_maxdepth(maxdepth)
{
  this->_load(is);
}

RangeData::RangeData(const char *fpath, const int maxdepth)
    throw(IOError,BadDataException)
    : 
      m_width(0),
      m_height(0),
      m_data(NULL),
      m_depthmap(NULL),
      m_maxdepth(maxdepth)
{
  std::ifstream ifs(fpath);
  if(ifs)
  {
    this->_load(ifs);
  }
  else
  {
    std::string msg("CANNOT OPEN: ");
    throw IOError((msg+fpath).c_str());
  }
  ifs.close();
}

// Destructor
RangeData::~RangeData(void)
{
  delete [] m_data;
  if(m_depthmap != NULL)
    delete [] m_depthmap;
}

// Copy Constructor
RangeData::RangeData(const RangeData &rd)
    : m_width(rd.getWidth()),
      m_height(rd.getHeight()),
      m_maxdepth(rd.getSensorMaxDepth())
{
  m_data = new double[m_width*m_height*3];
  rd.copyData(m_data);

  unsigned char *depthmap = rd.getDepthMap();
  if(depthmap == NULL)
    m_depthmap = NULL;
  else
  {
    m_depthmap = new unsigned char[m_width*m_height];
    memcpy(m_depthmap, depthmap, m_width*m_height*sizeof(unsigned char));
  }
}

RangeData &
RangeData::operator=(const RangeData &rd)
{
  if(this == &rd) return *this;

  int width = rd.getWidth();
  int height = rd.getHeight();

  if(width*height != m_width*m_height)
  {
    delete [] m_data;
    m_data = new double[width*height*3];
  }
  rd.copyData(m_data);

  unsigned char *depthmap = rd.getDepthMap();
  if(depthmap == NULL)
    m_depthmap = NULL;
  else
  {
    m_depthmap = new unsigned char[width*height];
    memcpy(m_depthmap, depthmap, width*height*sizeof(unsigned char));
  }

  m_width = width;
  m_height = height;
  m_maxdepth = rd.getSensorMaxDepth();

  return *this;
}

void
RangeData::setData(const double *data)
{
  memcpy(m_data, data, m_width*m_height*3*sizeof(double));
  return;
}

void
RangeData::setData(const float *data)
{
  int dlen = m_width*m_height*3;
  for(int i=0; i<dlen; i++)
  {
    m_data[i] = *data++;
  }
  return;
}

void
RangeData::setData(const float *xdata, const float *ydata, const float *zdata)
{
  int dlen = m_width*m_height;
  for(int i=0, j=0; i<dlen; i++, j+=3)
  {
    m_data[j] = xdata[i];
    m_data[j+1] = ydata[i];
    m_data[j+2] = zdata[i];
  }

  return;
}

void
RangeData::copyData(double *data) const
  throw(BadDataException)
{
  memcpy(data, m_data, m_width*m_height*3*sizeof(double));
  return;
}


void
RangeData::updateDepthMap(const double maxz)
  throw(BadDataException)
{
  if(m_depthmap == NULL)
    m_depthmap = new unsigned char[m_width*m_height];

  double scale = 253.0 / maxz;
  for(int i=0, j=2; i<m_width*m_height; i++, j+=3)
  {
    if(m_data[j] < 0.0)
    {
      int tmpdata = (int)m_data[j];
      if(tmpdata == ZVAL_TOOFAR)
        m_depthmap[i] = DEPTHMAP_NODATA;
      else  // ZVAL_INVALID or ZVAL_TOONEAR => FAILURE to DANGER
        m_depthmap[i] = DEPTHMAP_INVALID;
    }
    else
    {
      if(m_data[j] > maxz)
        m_depthmap[i] = DEPTHMAP_NODATA;
      else
      {
        m_depthmap[i] = (unsigned char)(254 - scale * m_data[j]);
        if(m_depthmap[i] == 0)
          throw UFV::RangeData::BadDataException("DEPTH OUT OF RANGE");
      }
    }
  }

  return;
}

/*********************************************************************
 * Read range data
 */
static bool
is_space_line(std::string &buf)
{
  for(unsigned int i=0; i<buf.size(); i++)
  {
    if(!std::isspace(buf[i]))
      return false;
  }
  
  return true;    /* 全ての文字がSPACE    */
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

template<typename T>
static void
read_ndata(T *ndata, std::istream &is, std::string &buf)
{
  int ret = read_nextline(is, buf);
  if(ret != UFV::OK)
  {
    if(ret == UFV::END_OF_FILE) throw "BAD NDATA PARAMS";
  }

  std::istringstream iss(buf);  // "%d"
  char c;

  iss >> c >> *ndata;     //  %d
  if(iss.fail() == true) throw "BAD NDATA PARAMS";

  return;
}

static void
read_data(double *dat, const int ndata, std::istream &is, std::string &buf)
{
  for(int i=0, j=0; i<ndata; i++, j+=3)
  {
    int ret = read_nextline(is, buf);
    if(ret != UFV::OK)
    {
      if(ret == UFV::END_OF_FILE)
        throw "BAD DATA";
    }

    std::istringstream iss(buf);  // " - [ %lf, %lf, %lf ]"
    char c;

    iss >> c;              // -
    iss >> c >> dat[j];    // [ %lf
    iss >> c >> dat[j+1];  // , %lf
    iss >> c >> dat[j+2];  // , %lf ]

    if(iss.fail() == true)
      throw "BAD RANGE DATA";
  }

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

void
RangeData::_load(std::istream &is)
{
  bool flag_width=false;         /* 必須情報     */
  bool flag_height=false;         /* 必須情報     */
  bool flag_data=false;         /* 必須情報     */
  bool have_separator=false;
  int ret = UFV::OK;

  std::string buf;

  while(1)
  {
    ret = read_nextline(is, buf);
    if(ret != UFV::OK)
    {
      if(ret == UFV::END_OF_FILE)
        throw UFV::RangeData::BadDataException("BAD FORMAT");
    }

    if(is_separator_line(buf))
    {
      have_separator = true;
      break;
    }
  }

  try
  {
    while(1)
    {
      if(buf.compare(0, strlen("NDATA:"), "NDATA:") == 0)
      {
        if(flag_width || flag_height) throw "BAD FORMAT";
        read_ndata(&m_width, is, buf);
        m_height = 1;
        flag_width = true;
        flag_height = true;
      }

      if(buf.compare(0, strlen("WIDTH:"), "WIDTH:") == 0)
      {
        if(flag_width) throw "BAD FORMAT";
        read_ndata(&m_width, is, buf);
        flag_width = true;
      }

      if(buf.compare(0, strlen("HEIGHT:"), "HEIGHT:") == 0)
      {
        if(flag_height) throw "BAD FORMAT";
        read_ndata(&m_height, is, buf);
        flag_height = true;
      }

      if(buf.compare(0, strlen("DATA:"), "DATA:") == 0)
      {
        if(!flag_width || !flag_height) // サイズ不明
          throw UFV::RangeData::BadDataException("BAD FORMAT");
        if(flag_data)  // データ重複
          throw UFV::RangeData::BadDataException("BAD FORMAT");

        int dlen = m_width*m_height;
        m_data = new double[dlen*3];
        read_data(m_data, dlen, is, buf);
        flag_data = true;
      }

      ret = read_nextline(is, buf);
      if(ret == UFV::END_OF_FILE || is_separator_line(buf))
      {
        if(!flag_data)
          throw "NO DATA.";
        break;
      }
    }
  }
  catch(char *ex)
  {
    if(m_data != NULL)
    {
      delete [] m_data;
      m_data = NULL;
    }
    throw UFV::RangeData::BadDataException(ex);
  }

  return;
}

/*********************************************************************
 * Print Checker Pattern
 */
int
RangeData::print(std::ostream &os, const char *indent) const
{
  os << indent << "WIDTH:" << std::endl;
  os << indent << "  - " << m_width << std::endl;

  os << indent << "HEIGHT:" << std::endl;
  os << indent << "  - " << m_height << std::endl;

  os << indent << "DATA:" << std::endl;
  for(int i=0; i<m_width*m_height; i++)
    os << indent
       << "  - [ " 
       << std::setw(10) << std::right << m_data[i*3]   << ", "
       << std::setw(10) << std::right << m_data[i*3+1] << ", "
       << std::setw(10) << std::right << m_data[i*3+2] << " ]" << std::endl;

  return UFV::OK;
}

/*
void
RangeData::setIntensities(const short *intensities, const int nchannels)
{
  int dlen = m_width*m_height*nchannels;

  if(!m_have_intensities)
  {
    m_intensities = new short[dlen];
    m_have_intensities = true;
  }
  else
  {
    if(m_nchannels != nchannels)
    {
      delete [] m_intensities;
      m_intensities = new short[dlen];
    }
  }
  
  m_nchannels = nchannels;

  memcpy(m_intensities, intensities, dlen*sizeof(short));

  return;
}

void
RangeData::setIntensities(const unsigned char *intensities,
                          const int nchannels)
{
  int dlen = m_width*m_height*nchannels;

  if(!m_have_intensities)
  {
    m_intensities = new short[dlen];
    m_have_intensities = true;
  }
  else
  {
    if(m_nchannels != nchannels)
    {
      delete [] m_intensities;
      m_intensities = new short[dlen];
    }
  }

  m_nchannels = nchannels;
  
  for(int i=0; i<dlen; i++)
    m_intensities[i] = intensities[i];

  return;
}

void
RangeData::getIntensities(short *intensities) const
  throw(BadDataException)
{
  if(!m_have_intensities)
    throw UFV::RangeData::BadDataException("NO INTENSITY DATA");
  
  memcpy(intensities, m_intensities,
         m_width*m_height*m_nchannels*sizeof(short));

  return;
}

void
RangeData::updateIntensityMap(const short maxintensity,
                              const unsigned char satulatedpixel)
  throw(BadDataException)
{
  if(!m_have_intensities)
    throw UFV::RangeData::BadDataException("NO INTENSITY DATA");
  
  int dlen = m_width*m_height*m_nchannels;

  if(m_intensitymap == NULL)
    m_intensitymap = new unsigned char[dlen];

  if(maxintensity > 255)
  {
    double scale = 255.0/maxintensity;
    for(int i=0; i<dlen; i++)
    {
      if(m_intensities[i]<0)
        m_intensitymap[i] = satulatedpixel;
      else
        m_intensitymap[i] = (unsigned char)(m_intensities[i]*scale);
    }
  }
  else
  {
    for(int i=0; i<dlen; i++)
    {
      if(m_intensities[i]<0)
        m_intensitymap[i] = satulatedpixel;
      else
        m_intensitymap[i] = (unsigned char)(m_intensities[i]);
    }
  }

  return;
}
*/
