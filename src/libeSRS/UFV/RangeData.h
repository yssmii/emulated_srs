// -*- C++ -*-
/** 
 * @file RangeData.h
 * @brief UFV::RangeData class definition
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2011  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __LIBESRS_UFV_RANGEDATA_H__
#define __LIBESRS_UFV_RANGEDATA_H__

#include <iostream>
#include <string>
#include <stdexcept>

#include "UFV/types.h"

namespace UFV 
{

class RangeData
{
public:
  static const int ZVAL_INVALID = -1;        // 計測不能
  static const int ZVAL_TOONEAR = -2;        // 計測範囲外（近すぎ）
  static const int ZVAL_TOOFAR = -10;        // 計測範囲外（遠すぎ）

  static const int DEPTHMAP_INVALID = 255;   // 計測不能画素、故障画素
  static const int DEPTHMAP_NODATA = 0;      // データなし

  static const double DEFAULT_MAXDEPTH;      // mm

public:
  class IOError : public std::ios_base::failure
  {
  public:
    IOError(const std::string &msg) : std::ios_base::failure(msg) {};
  };
  class BadDataException : public std::domain_error
  {
  public:
    BadDataException(const std::string &msg) : std::domain_error(msg) {};
  };

public:
  RangeData(const char *path, const int maxdepth=DEFAULT_MAXDEPTH)
    throw(IOError, BadDataException);
  RangeData(std::istream &is, const int maxdepth=DEFAULT_MAXDEPTH)
    throw(BadDataException);
  RangeData(const int width, const int height,
            const int maxdepth=DEFAULT_MAXDEPTH);
  
  virtual ~RangeData(void);

  RangeData(const RangeData &rd);
  RangeData& operator=(const RangeData &rd);

public:
  int print(std::ostream &os, const char *indent="") const;

public:
  //! DepthMapの幅
  int width(void) const { return this->getWidth(); };
  int getWidth(void) const { return m_width; };

  //! DepthMapの高さ
  int height(void) const { return this->getHeight(); };
  int getHeight(void) const { return m_height; };

public:
  void setSensorMaxDepth(const double val) { m_maxdepth = val; };
  double getSensorMaxDepth(void) const { return m_maxdepth; };

public:
  //! x0,y0,z0,x1,y1,z1,...
  //! zが負値=無効データ
  void setData(const double *data);
  void setData(const float *data);
  void setData(const float *xdata, const float *ydata, const float *zdata);

  //! 直接データポインタが返る。取り扱い注意
  double *getData(void) const { return m_data; };
  double *data(void) const { return this->getData(); };
  
  //! データをコピー。引数には必要なサイズの領域が確保されていること
  void copyData(double *data) const throw(BadDataException);

public:
  /**
   * DepthMapを更新する.
   *
   * xyzからDepthMapを更新する。
   * この関数がコールされない限り、DepthMapは変更されない。
   * DepthMapの画素値 1..254
   *  有効画素: 1~254 (254 - Z*253/maxz, if Z <= maxz, 遠いほど白)
   *  計測エリア外: DEPTHMAP_TOOFAR=0 (目立たせるためあえて黒)
   *  計測不能画素: DEPTHMAP_INVALUD=255 (目立たせるためあえて白)
   * 
   * @param maxz            threshold for maximum depth value (Z)
   */
  void updateDepthMap(const double maxz) throw(BadDataException);
  unsigned char* getDepthMap(void) const { return m_depthmap; };

private:
  void _load(std::istream &is);

private:
  int m_width;  // DepthMapの width
  int m_height; // DepthMapの height

  /**
   * xyz座標データ
   * サイズは m_width * m_height * 3
   * DepthMapの左上から右下に向かって x0,y0,z0,x1,y1,z1,... の順
   * 座標系
   *   - 右手系
   *   - カメラ光学中心が原点
   *   - 光軸方向がz
   *   - 下方向がy
   * TODO: 各VBPDクラスの座標系が間違いなく実装されているか確認する #448
   */
  double *m_data;

  //! 計測に使用したセンサ固有の最大計測距離
  double m_maxdepth;

  /**
   * xyz座標データを uchar画像に投影した画像データ
   * updateDepthMap()をコールすると更新される
   */
  unsigned char *m_depthmap;

private:
  RangeData(void) {};
};

} // namespace UFV

#endif
