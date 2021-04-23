/**
 * Copyright (C) 2011  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#include <string.h>

#include "UFV/types.h"
#include "UFV/ImageData.h"

//#define DEBUG 0


/*
UFV::ImageData::ImageData(void)
    : m_width(0),
      m_height(0),
      m_nChannels(0),
      m_data(0)
{
}

UFV::ImageData::ImageData(const int width, const int height,
                          const int nChannels,
                          unsigned char* data)
    : m_width(width),
      m_height(height),
      m_nChannels(nChannels)
{
  m_data = new unsigned char[m_width*m_height*m_nChannels];
  if(data==0)
    memset(m_data, 0, m_width*m_height*m_nChannels);
  else
    memcpy(m_data, data, m_width*m_height*m_nChannels);
}

UFV::ImageData::~ImageData(void)
{
  if(m_data != 0) delete [] m_data;
}

// Copy Constructor
UFV::ImageData::ImageData(const UFV::ImageData &img)
    : m_width(img.width()),
      m_height(img.height()),
      m_nChannels(img.nChannels())
{
  m_data = new unsigned char[m_width*m_height*m_nChannels];
  img.copyData(m_data);
}

UFV::ImageData &
UFV::ImageData::operator=(const UFV::ImageData &img)
{
  if(this == &img) return *this;

  int width = img.width();
  int height = img.height();
  int nChannels = img.nChannels();

  if(m_width != width || m_height != height || m_nChannels != nChannels)
  {
    if(m_data != 0) delete [] m_data;
    m_data = new unsigned char[width*height*nChannels];
  }

  m_width = width;
  m_height = height;
  m_nChannels = nChannels;

  img.copyData(m_data);

  return *this;
}

void
UFV::ImageData::setData(const int width, const int height, 
                        const int nChannels,
                        const unsigned char *data)
{
  if(m_width != width || m_height != height || m_nChannels != nChannels)
  {
    if(m_data != 0) delete [] m_data;
    m_data = new unsigned char[width*height*nChannels];
  }

  m_width = width;
  m_height = height;
  m_nChannels = nChannels;

  if(data != 0)
    memcpy(m_data, data, m_width*m_height*m_nChannels);
  else
    memset(m_data, 0, m_width*m_height*m_nChannels);

  return;
}

unsigned char*
UFV::ImageData::getData(const int x, const int y) const
{
  return m_data + m_width*y + x;
}

void
UFV::ImageData::copyData(unsigned char *data) const
{
  memcpy(data, m_data, m_width*m_height*m_nChannels);
  return;
}

unsigned char
UFV::ImageData::getPixel(const int x, const int y) const
{
  return *(m_data + m_width*y + x);
}

UFV::ImageData
UFV::ImageData::cvtGray(void) const
{
  if(this->nChannels() == 1) return *this;

  UFV::ImageData gray(this->width(), this->height(), 1, 0);

  unsigned char *gdata = gray.data();
  int len = this->width() * this->height();
  for(int i=0, j=0; i<len; ++i)
  {
    double b = m_data[j++] * 0.114;
    double g = m_data[j++] * 0.587;
    double r = m_data[j++] * 0.299;
    int y = (int)(b+g+r);
    if(y > 255) y = 255;
    gdata[i] = (unsigned char)y;
  }

  return gray;
}

UFV::ImageData
UFV::ImageData::cvtBGR(void) const
{
  if(this->nChannels() == 3) return *this;

  UFV::ImageData bgr(this->width(), this->height(), 3, 0);

  unsigned char *bgrdata = bgr.data();
  int len = this->width() * this->height();
  for(int i=0, j=0; i<len; ++i)
  {
    bgrdata[j++] = m_data[i];
    bgrdata[j++] = m_data[i];
    bgrdata[j++] = m_data[i];
  }

  return bgr;
}
*/
