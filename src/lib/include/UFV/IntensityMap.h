// -*- C++ -*-
/** 
 * @file IntensityMap.h
 * @brief UFV::IntensityMap class definition
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2018  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __LIBESRS_UFV_INTENSITYMAP_H__
#define __LIBESRS_UFV_INTENSITYMAP_H__

#include "UFV/types.h"
#include "UFV/ImageData.h"

namespace UFV 
{

class IntensityMap
{
public:
  //! 輝度画像のタイプ
  // 必要に応じて追加すること
  typedef enum {
    PROP_INTENSITY,
    PROP_INTENSITY_LEFT,
    PROP_INTENSITY_RIGHT,
    PROP_DEPTH,
    PROP_NORMALS,
    PROP_CONFIDENCE,
    PROP_NUM // PROP数。ストッパー
  } Prop;

  //! 輝度画像のデータ型
  // 必要に応じて追加すること
  typedef enum {
    TYPE_U8,
    TYPE_S16,
    TYPE_F32,
    TYPE_D64,
    TYPE_NUM // TYPE数。ストッパー
  } Type;
  
public:
  //! デフォルトコンストラクタは、画像領域を確保しない
  IntensityMap(void);
  ~IntensityMap(void)
    {
      if(m_image != 0) delete m_image;
    };

  //! コピーコンストラクタ
  // - データ領域はコピーされる
  IntensityMap(const IntensityMap &im);

  //! 代入演算子
  // - データ領域は上書きされる。
  // - 型・サイズが異なる場合、データ領域はdelete＆再確保される。
  IntensityMap& operator=(const IntensityMap &im);

public:
  //! コンストラクタ
  // - 新しいデータ領域が確保される。データの値は不定
  IntensityMap(const int width, const int height, const int nchannels,
               Type type);

  //! コンストラクタ
  // - 新しいデータ領域が確保され、値がコピーされる
  template <typename T>
  IntensityMap(const int width, const int height, const int nchannels,
               const T *pdata);

  //! コンストラクタ
  // - 新しいデータ領域が確保され、値がコピーされる
  template <typename T>
  IntensityMap(const UFV::ImageData<T> &img);

public:
  virtual void reshape(const int width, const int height, const int nchannels,
                       Type type);
public:
  Type type(void) const { return m_type; };
  Type getType(void) const { return this->type(); };
  
public:
  Prop prop(void) const { return m_prop; };
  Prop getProp(void) const { return this->prop(); };
  Prop setProp(const Prop prop) { return m_prop = prop; };

public:
  /*! データをセットする1
   * - pdataを m_imageに memcpy するので、
   *   時短のためには m_image のデータを直接上書きしたほうがいい
   */
  template<typename T>
  void setData(const int width, const int height, const int nchannels,
               const T *pdata);

  /*! データをセットする2
   * - imgの内容をm_imageにコピーするので、
   *   時短のためには m_image のデータを直接上書きしたほうがいい
   */
  template<typename T>
  void setData(const UFV::ImageData<T> &img);

  /*! データ領域のポインタ
   * - 直接データアクセス
   * - クラスの内部データを破壊可能なので取り扱い注意
   * - リリース時には、適切に修正されるべき
   */
  template<typename T>
  T* getData(void) const {
    if(m_image == 0)
      return 0;
    else
      return ((UFV::ImageData<T>*)m_image)->getData();
  };
  
  /*! ImageDataのポインタ
   * - 直接データアクセス
   * - クラスの内部データを破壊可能なので取り扱い注意
   * - リリース時には、適切に修正されるべき
   */
  template<typename T>
  UFV::ImageData<T> *getImageData(void) const {
      return ((UFV::ImageData<T>*)m_image);
  };
  
public:
  int width(void) const {
    if(m_image == 0)
      return 0;
    else
      return m_image->width();
  };
  int height(void) const {
    if(m_image == 0)
      return 0;
    else
      return m_image->height();
  };
  int nchannels(void) const {
    if(m_image == 0)
      return 0;
    else
      return m_image->nchannels();
  };

public:
  int normalize(UFV::ImageData<unsigned char> &ret_dimg,
                const double maxval=255.0) const;

private:
  Type transType(const unsigned char *) { return TYPE_U8; };
  Type transType(const short *) { return TYPE_S16; };
  Type transType(const float *) { return TYPE_F32; };
  Type transType(const double *) { return TYPE_D64; };

private:
  Type m_type;
  Prop m_prop;
  UFV::ImageBase *m_image;
};

template <typename T>
IntensityMap::IntensityMap(
  const int width, const int height, const int nchannels,
  const T *pdata)
    : m_prop(PROP_INTENSITY)
{
  m_image = new UFV::ImageData<T>(width, height, nchannels, pdata);
  m_type = transType(pdata);
}

template <typename T>
IntensityMap::IntensityMap(const UFV::ImageData<T> &img)
    : m_prop(PROP_INTENSITY)
{
  m_image = new UFV::ImageData<T>(img);
  m_type = transType(img.getData());
}

template<typename T>
void
IntensityMap::setData(
  const int width, const int height, const int nchannels,
  const T *pdata)
{
  Type type = transType(pdata);
    
  if(width != this->width() || height != this->height() ||
     nchannels != this->nchannels() || type != this->type())
  {
    if(m_image != 0) delete m_image;
    m_image = new UFV::ImageData<T>(width,height,nchannels);
  }
  
  m_type = type;

  ((UFV::ImageData<T>*)m_image)->setData(width, height, nchannels, pdata);
  
  return;
}

template<typename T>
void
IntensityMap::setData(const UFV::ImageData<T> &img)
{
  T *pdata=img.getData();
  Type type = transType(pdata);
  int width = img.width();
  int height = img.height();
  int nchannels = img.nchannels();
  //*m_image = img;

  if(width != this->width() || height != this->height() ||
     nchannels != this->nchannels() || type != this->type())
  {
    if(m_image != 0) delete m_image;
    m_image = new UFV::ImageData<T>(width,height,nchannels);
  }

  m_type = type;

  ((UFV::ImageData<T>*)m_image)->setData(width, height, nchannels, pdata);
  
  return;
}

}

#endif
