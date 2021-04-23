// -*- C++ -*-
/**
 * @file ImageData.h
 * @brief ImageData class definition
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2011  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __LIBESRS_UFV_IMAGEDATA_H__
#define __LIBESRS_UFV_IMAGEDATA_H__

#include <string.h>
#include <math.h>
#include <float.h>
#include <string>
#include <opencv2/opencv.hpp> 
#include <opencv2/highgui/highgui.hpp> 

#include "UFV/types.h"
#include "UFV/utils.h"
#include "UFV/keydefs.h"

namespace UFV
{

class ImageBase
{
public:
  ImageBase(void) : m_width(0), m_height(0), m_nchannels(0) {};
  ImageBase(const int w, const int h, const int n)
      : m_width(w), m_height(h), m_nchannels(n) {};
  virtual ~ImageBase(void) {};
  
public:
  int width(void) const { return m_width; };
  int height(void) const { return m_height; };
  int nchannels(void) const { return m_nchannels; };

public:
  bool isEmpty(void) const { return true; };
  
protected:
  int m_width;
  int m_height;
  int m_nchannels;
};

template <class T>
class ImageData : public ImageBase
{
public:
  ImageData(void) : ImageBase(), m_p_write(false), m_data(0){};
  ImageData(const int width, const int height, const int nchannels=1,
            const T *data=0);
  /*! TODO: 型を保持して、unsigned int のときは normalize しないようにする
  ImageData(const int width, const int height, const int nchannels=1,
            const UFV::Type type, const void *data=0);
  */
  virtual ~ImageData(void);

public:  
  ImageData(const ImageData<T> &ib);
  template <typename U> ImageData(const ImageData<U> &ib);

  /*!
   * @if jp
   *
   * @brief データ型が同じ場合の代入演算子
   *
   * データサイズが同じならば、データをコピー。
   * 同じでなければ、データ領域を開放＆再取得した後、データをコピー
   *
   * @else
   *
   * @brief An assignment operator
   *
   * @endif
   */
  ImageData& operator=(const ImageData<T> &ib);

  /*!
   * @if jp
   *
   * @brief データ型が異なる場合の代入演算子
   *
   * データ領域を開放＆再取得した後、データをコピーする
   *
   * @else
   *
   * @brief An aassignment operator
   *
   * @endif
   */
  template <typename U> ImageData<T>& operator=(const ImageData<U> &ib);

public:
  bool isEmpty(void) const { return m_data == 0; };

public:
  void setData(const int width, const int height, const int nchannels=1,
               const T *data=0);
  void reshape(const int width, const int height, const int nchannels=1) {
    this->setData(width, height, nchannels);
  };

public:
  T* getData(const int x=0, const int y=0, const int c=0) const;
  T* data(const int x=0, const int y=0, const int c=0) const {
    return this->getData(x,y,c);
  };

  void copyData(T *data) const;
  template <typename U> void copyData(U *data) const;

  T getPixel(const int x, const int y, const int c=0) const;

  void fillData(T val);

public:
  bool haveShape(const int w, const int h, const int n) const {
    return m_width == w && m_height == h && m_nchannels == n;
  };
  bool haveShape(const ImageData &img) const {
    return m_width == img.width() && m_height == img.height() &&
      m_nchannels == img.nchannels();
  };

public:
  virtual int normalize(UFV::ImageData<unsigned char> &img,
                        const double maxval) const;

  virtual int display(const std::string wlabel, const int msec,
                      const double maxval=255.0) const;
  virtual int display(const std::string wlabel, const int msec,
                      const UFV::Rect &rect,  // 表示範囲 ROI
                      const double mag,  // 表示倍率、0以下なら何もしない
                      const double maxval=255.0) const;

  virtual int displayRaw(const std::string wlabel, const int msec) const;

public:
  virtual int convertBGR(UFV::ImageData<T> &img) const;
  virtual int convertGray(UFV::ImageData<T> &img,
                          const double maxval=255.0) const;

public:
  void setWriteImage(const bool set, const std::string dir) {
    m_p_write = set; m_dir = dir;
  };
  void setWriteImage(bool set){ m_p_write = set; };
  bool getWriteImage(void) const { return m_p_write; };

  std::string getWriteDirectory(void) const { return m_dir; };
  void setWriteDirectory(std::string dir){ m_dir = dir; };

  virtual void writeImage(const std::string name="", // ファイル名+拡張子
                          const double maxval=255.0) const;

  virtual void writeImage(const UFV::Rect &rect,   // 表示範囲 ROI
                          const double mag=-1.0, // 倍率、0以下なら何もしない
                          const std::string name="", // ファイル名+拡張子
                          const double maxval=255.0) const;
private:
  bool m_p_write;
  std::string m_dir;

private:
  T *m_data;  // 連続していること
};

template<class T>
ImageData<T>::ImageData(const int w, const int h, const int n, const T* data)
    : ImageBase(w, h, n), m_p_write(false), m_data(0)
{
  if(w <= 0 || h <= 0 || n <= 0)
  {
    m_width = 0;
    m_height = 0;
    m_nchannels = 0;
    m_data = 0;
  }
  else
  {
    m_data = new T[w*h*n];
    if(data==0)
      memset(m_data, 0, w*h*n*sizeof(T));
    else
      memcpy(m_data, data, w*h*n*sizeof(T));
  }
}

template<class T>
ImageData<T>::~ImageData(void)
{
  if(m_data != 0) delete [] m_data;
}
  
// Copy Constructor
template<class T>
ImageData<T>::ImageData(const ImageData<T> &img)
    : ImageBase(img.width(), img.height(), img.nchannels())
{
  m_data = new T[img.width()*img.height()*img.nchannels()];
  img.copyData(m_data);
}

template<class T> template<typename U>
ImageData<T>::ImageData(const ImageData<U> &img)
    : ImageBase(img.width(), img.height(), img.nchannels())
{
  m_data = new T[img.width()*img.height()*img.nchannels()];
  img.copyData(m_data);
}

template<class T>
ImageData<T> &
ImageData<T>::operator=(const ImageData<T> &img)
{
  if(this == &img) return *this;

  int width = img.width();
  int height = img.height();
  int nchannels = img.nchannels();

  if(this->width() != width || this->height() != height ||
     this->nchannels() != nchannels)
  {
    if(m_data != 0) delete [] m_data;
    m_data = new T[width*height*nchannels];
  }

  m_width = width;
  m_height = height;
  m_nchannels = nchannels;

  img.copyData(m_data);

  return *this;
}

template<class T> template <typename U>
ImageData<T> &
ImageData<T>::operator=(const ImageData<U> &img)
{
  m_width = img.width();
  m_height = img.height();
  m_nchannels = img.nchannels();

  if(m_data != 0) delete [] m_data;
  m_data = new T[m_width*m_height*m_nchannels];

  img.copyData(m_data);

  return *this;
}

template<class T>
void
ImageData<T>::setData(const int w, const int h, const int n,
                      const T *data)
{
  if(m_width != w || m_height != h || m_nchannels != n)
  {
    if(m_data != 0) delete [] m_data;
    m_data = new T[w*h*n];

    m_width = w;
    m_height = h;
    m_nchannels = n;
  }

  if(data != 0)
    memcpy(m_data, data, m_width*m_height*m_nchannels*sizeof(T));
  /*
  else
    memset(m_data, 0, m_width*m_height*m_nchannels*sizeof(T));
  */
  
  return;
}

template<class T>
T*
ImageData<T>::getData(const int x, const int y, const int c) const
{
  if(m_data == 0) return 0;
  return m_data + (m_width*y + x)*m_nchannels + c;
}

template<class T>
void
ImageData<T>::copyData(T *data) const
{
  memcpy(data, m_data, m_width*m_height*m_nchannels*sizeof(T));
  return;
}

template<class T> template <typename U>
void
ImageData<T>::copyData(U *data) const
{
  T *pdat = m_data;

  if(m_nchannels > 1)
  {
    for(int i=0; i<m_height; ++i)
      for(int j=0; j<m_width; ++j)
        for(int k=0; j<m_nchannels; ++k)
          *data++ = static_cast<U>(*pdat++);
  }
  else
  {
    for(int i=0; i<m_height; ++i)
      for(int j=0; j<m_width; ++j)
        *data++ = static_cast<U>(*pdat++);
  }
  return;
}

template<class T>
T
ImageData<T>::getPixel(const int x, const int y, const int c) const
{
  return *(m_data + (m_width*y + x)*m_nchannels + c);
}

template<class T>
void
ImageData<T>::fillData(T val)
{
  T *pdat = m_data;
  for(int i=0; i<m_width*m_height*m_nchannels; ++i)
  {
    *pdat++ = val;
  }
  
  return;
}

template<class T>
int
ImageData<T>::normalize(ImageData<unsigned char> &img,
                        const double maxval) const
{
  if(fabs(maxval) < FLT_EPSILON)
    return UFV::NG;

  if(this->width() != img.width() || this->height() != img.height() ||
     this->nchannels() != img.nchannels())
    return UFV::NG;

  T *pdat = m_data;
  unsigned char *pimg = img.data();
  double scale = 255.0 / maxval;
  for(int i=0; i<m_width*m_height*m_nchannels; ++i)
  {
    if(*pdat > maxval)
      *pimg = 255;
    else
      *pimg = (*pdat) * scale;
    pimg++;
    pdat++;
  }

  return UFV::OK;
}

template<class T>
int
ImageData<T>::convertBGR(ImageData<T> &img) const
{
  if(this->width() != img.width() || this->height() != img.height() ||
     this->nchannels() != 1 ||
     img.nchannels() != 3)
    return UFV::NG;

  T *bgrdata = img.data();
  int len = this->width() * this->height();
  for(int i=0, j=0; i<len; ++i)
  {
    bgrdata[j++] = m_data[i];
    bgrdata[j++] = m_data[i];
    bgrdata[j++] = m_data[i];
  }

  return UFV::OK;
}


template<class T>
int
ImageData<T>::convertGray(ImageData<T> &img, double maxval) const
{
  if(this->width() != img.width() || this->height() != img.height() ||
     this->nchannels() != 3 ||
     img.nchannels() != 1)
    return UFV::NG;

  T *gdata = img.data();
  int len = this->width() * this->height();
  for(int i=0, j=0; i<len; ++i)
  {
    double b = m_data[j++] * 0.114;
    double g = m_data[j++] * 0.587;
    double r = m_data[j++] * 0.299;
    double y = b+g+r;
    if(y > maxval) y = maxval;
    gdata[i] = (T)y;
  }

  return UFV::OK;
}

template<class T>
int
ImageData<T>::display(const std::string wlabel, const int msec,
                      const double maxval) const
{
  UFV::ImageData<unsigned char> showimg(this->width(),
                                        this->height(), this->nchannels());
  int ret = this->normalize(showimg, maxval);
  if(ret != UFV::OK) return ret;

  int itype = (this->nchannels() == 3) ? CV_8UC3 : CV_8U;
  cv::Mat simg(cv::Size(this->width(), this->height()), itype,
               showimg.data());
  cv::imshow(wlabel, simg);
  if(msec >= 0)
  {
    int kret = cv::waitKey(msec);
    if(kret == XK_q)
      return UFV::END_OF_FILE;
    else if(kret == XK_space)
      cv::waitKey(0);
  }

  return UFV::OK;
}

template<class T>
int
ImageData<T>::display(const std::string wlabel, const int msec,
                      const UFV::Rect &rect, const double mag,
                      const double maxval) const
{
  UFV::ImageData<unsigned char> showimg(this->width(), this->height(),
                                        this->nchannels());
  int ret = this->normalize(showimg, maxval);
  if(ret != UFV::OK) return ret;

  int itype = (this->nchannels() == 3) ? CV_8UC3 : CV_8U;
  cv::Mat simg(cv::Size(this->width(), this->height()), itype,
               showimg.data());

  cv::Rect roi(rect.x, rect.y, rect.width, rect.height);

  if(mag<FLT_EPSILON || fabs(mag-1.0) < FLT_EPSILON)
    cv::imshow(wlabel, simg(roi));
  else
  {
    cv::Mat dst;
    cv::resize(simg(roi), dst, cv::Size(), mag, mag, cv::INTER_NEAREST);
    cv::imshow(wlabel, dst);
  }

  if(msec >= 0)
  {
    int kret = cv::waitKey(msec);
    if(kret == XK_q)
      return UFV::END_OF_FILE;
    else if(kret == XK_space)
      cv::waitKey(0);
  }

  return UFV::OK;
}

template<class T>
int
UFV::ImageData<T>::displayRaw(const std::string wlabel,
                           const int msec) const
{
  int itype = (this->nchannels() == 3) ? CV_8UC3 : CV_8U;
  cv::Mat simg(cv::Size(this->width(), this->height()), itype,
               this->m_data);

  cv::imshow(wlabel, simg);
  if(msec >= 0)
  {
    int kret = cv::waitKey(msec);
    if(kret == XK_q)
      return UFV::END_OF_FILE;
    else if(kret == XK_space)
      cv::waitKey(0);
  }

  return UFV::OK;
}


template<class T>
void
ImageData<T>::writeImage(const std::string name, const double maxval) const
{
  if(m_p_write != true) return;

  std::string file_name;

  if(name != ""){
    file_name = m_dir + name;
  }else{
	  //! 画像名の指定がないときは、現在時刻＋BMP形式で保存
	  file_name = m_dir + UFV::getLocalTimeString() + ".bmp";
  }

  //std::cerr << file_name << std::endl;
      
  /* !２０１８防災科研実験用。ノーマライズ不要
  UFV::ImageData<unsigned char> showimg(this->width(),
                                        this->height(), this->nchannels());
  int ret = this->normalize(showimg, maxval);
  if(ret != UFV::OK) return;

  int itype = (this->nchannels() == 3) ? CV_8UC3 : CV_8U;
  cv::Mat simg(cv::Size(this->width(), this->height()), itype,
               showimg.data());
  */
  int itype = (this->nchannels() == 3) ? CV_8UC3 : CV_8U;
  cv::Mat simg(cv::Size(this->width(), this->height()), itype,
               this->data());
  cv::imwrite(file_name, simg);

  return;
}

template<class T>
void
ImageData<T>::writeImage(const UFV::Rect &rect, const double mag,
                         const std::string name, const double maxval) const
{
  if(m_p_write != true) return;

  UFV::ImageData<unsigned char> showimg(this->width(), this->height(),
                                        this->nchannels());
  int ret = this->normalize(showimg, maxval);
  if(ret != UFV::OK) return;

  std::string file_name;

  if(name != ""){
    file_name = m_dir + name;
  }else{
	//! 画像名の指定がないときは、現在時刻＋BMP形式で保存
    file_name = m_dir + UFV::getLocalTimeString() + ".bmp";
  }

  int itype = (this->nchannels() == 3) ? CV_8UC3 : CV_8U;
  cv::Mat simg(cv::Size(this->width(), this->height()), itype,
               showimg.data());
  cv::Rect roi(rect.x, rect.y, rect.width, rect.height);

  if(mag<FLT_EPSILON || fabs(mag-1.0) < FLT_EPSILON)
    cv::imwrite(file_name, simg(roi));
  else
  {
    cv::Mat dst;
    cv::resize(simg(roi), dst, cv::Size(), mag, mag, cv::INTER_NEAREST);
    cv::imwrite(file_name, dst);
  }

  return;
}

} // namespace UFV

#endif
