// -*- C++ -*-
/** 
 * @file DetectionMap.h
 * @brief bas
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2013  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __LIBESRS_ESRS_DETECTIONMAP_H__
#define __LIBESRS_ESRS_DETECTIONMAP_H__

#include <stdio.h>
#include <vector>

#include "UFV/types.h"

#include "eSRS/Object.h"
#include "eSRS/DepthMap.h"

namespace eSRS
{
/*! @class DetectionMap
 *  @brief DetectionMapクラス
 * 
 *  Detection Map
 */
class DetectionMap : public DepthMap
{
public:

  //! Zキーの閾値距離 (mm)
  static const double DEFAULT_ZKEY;

  //! これより離れていれば別物体 (mm)
  static const double DEFAULT_MIN_GAP;

  //! 物体の最小サイズ (pixel)
  static const int DEFAULT_MIN_SIZE=500;

private:

  //! ギャップのピクセル値
  static const int PIXELVALUE_GAP = -100;

public:
  /*!
   *  @brief コンストラクタ
   *  @note       N/A
   *  @attention  N/A
   */
  DetectionMap(void) :
      m_zkey(DEFAULT_ZKEY),
      m_min_gap(DEFAULT_MIN_GAP),
      m_min_size(DEFAULT_MIN_SIZE),
      m_flg_drawlabel(true) {};

  /*!
   *  @brief コンストラクタ
   *  @param  [in] width  基底クラスに渡すwidth
   *  @param  [in] height  基底クラスに渡すheight
   *  @param  [in] data  基底クラスに渡すdata
   *  @param  [in] zkey  Zキー
   *  @param  [in] mingap  ギャップの最小値
   *  @param  [in] minsize  物体の最小サイズ
   *  @note       N/A
   *  @attention  N/A
   */
  DetectionMap(const int width, const int height, const float *data=0,
               const double zkey=DEFAULT_ZKEY,
               const double mingap=DEFAULT_MIN_GAP,
               const int minsize=DEFAULT_MIN_SIZE) :
      eSRS::DepthMap(width, height, data),
      m_zkey(zkey),
      m_min_gap(mingap),
      m_min_size(minsize),
      m_flg_drawlabel(true),
      m_labeled(width, height, 1) {};

  /*!
   *  @brief コンストラクタ
   *  @param  [in] imgData  基底クラスに渡すImageData
   *  @param  [in] zkey  Zキー
   *  @param  [in] mingap  ギャップの最小値
   *  @param  [in] minsize  物体の最小サイズ
   *  @note       N/A
   *  @attention  N/A
   */
  template<class T> DetectionMap(const UFV::ImageData<T> &imgData,
                                 const double zkey=DEFAULT_ZKEY,
                                 const double mingap=DEFAULT_MIN_GAP,
                                 const int minsize=DEFAULT_MIN_SIZE) :
      eSRS::DepthMap(imgData),
      m_zkey(zkey),
      m_min_gap(mingap),
      m_min_size(minsize),
      m_flg_drawlabel(true),
      m_labeled(imgData.width(), imgData.height(), 1)
    {
      m_obs_vec.clear();
    };

  /*!
   *  @brief コンストラクタ
   *  @param  [in] depthMap  基底クラスに渡すDepthMap
   *  @param  [in] zkey  Zキー
   *  @param  [in] mingap  ギャップの最小値
   *  @param  [in] minsize  物体の最小サイズ
   *  @note       N/A
   *  @attention  N/A
   */
  DetectionMap(const eSRS::DepthMap &depthMap,
               const double zkey=DEFAULT_ZKEY,
               const double mingap=DEFAULT_MIN_GAP,
               const int minsize=DEFAULT_MIN_SIZE) :
      eSRS::DepthMap(depthMap),
      m_zkey(zkey),
      m_min_gap(mingap),
      m_min_size(minsize),
      m_flg_drawlabel(true),
      m_labeled(depthMap.width(), depthMap.height(), 1)
    {
      m_obs_vec.clear();
    };

  /*!
   *  @brief デストラクタ
   *  @note       N/A
   *  @attention  N/A
   */
  virtual ~DetectionMap(void) {};

public:

  virtual void reshape(const int width, const int height) {
    this->eSRS::DepthMap::reshape(width, height);
    m_labeled.reshape(width, height, 1);
    m_obs_vec.clear();
    return;
  };

public:
  /*!
   *  @brief  Zキー設定
   *  @param  [in] zkey  設定するZキー
   *  @note
   *    - メンバ変数 m_zkeyに zkeyを代入する
   *  @attention  N/A
   */
  void setZkey(const double zkey) { m_zkey = zkey; };

  /*!
   *  @brief  Zキー取得
   *  @retval double  Zキー
   *  @note
   *    - メンバ変数 m_zkeyを返す
   *  @attention  N/A
   */
  double getZkey(void) const { return m_zkey; };

  /*!
   *  @brief  ギャップの最小値設定
   *  @param  [in] mingap  設定するギャップの最小値
   *  @note
   *    - メンバ変数 m_min_gapに mingapを代入する
   *  @attention  N/A
   */
  void setMinGap(const double mingap) { m_min_gap = mingap; };
  
  /*!
   *  @brief  ギャップの最小値取得
   *  @retval double  ギャップの最小値
   *  @note
   *    - メンバ変数 m_min_gapを返す
   *  @attention  N/A
   */
  double getMinGap(void) const { return m_min_gap; };

  /*!
   *  @brief  物体の最小サイズ設定
   *  @param  [in] minsize  設定する物体の最小サイズ
   *  @note
   *    - メンバ変数 m_min_sizeに minsizeを代入する
   *  @attention  N/A
   */
  void setMinSize(const int minsize) { m_min_size = minsize; };
  
  /*!
   *  @brief  物体の最小サイズ取得
   *  @retval int  物体の最小サイズ
   *  @note
   *    - メンバ変数 m_min_sizeを返す
   *  @attention  N/A
   */
  int getMinSize(void) const { return m_min_size; };

  /*!
   *  @brief  display()で物体ラベルを表示するかどうか
   *  @param [in] flg  表示するときtrue。デフォルトはtrue
   *  @note
   *    - メンバ変数 m_min_sizeを返す
   *  @attention  N/A
   */
  void setDrawLabel(const bool flg) { m_flg_drawlabel = flg; };

public:

  /*!
   *  @brief  物体検出
   *  @retval int  検出数
   *  @note
   *    - 物体を検出して obstacle情報を作成する
   *  @attention
   *    - 物体を識別してobstacleオブジェクトとして保持する。
   *      現状、obstacleオブジェクトのサイズ(BoundingVolume)計算の際に
   *      DetectionMapに設定されているFocal Lengthを使用している。
   *      本来は、RangerImplの実装クラスにて3次元座標点を計算し、
   *      RangeDataに設定しているため、その値を使用するのがよい。
   */
  virtual int detect(void);
  
  /*!
   *  @brief  画像表示
   *  @param  [in] wlabel  ラベル
   *  @param  [in] msec  タイムアウト値
   *  @retval UFV::OK  正常終了
   *  @note
   *    - 画像を表示する
   *  @attention  N/A
   */
  virtual int display(const std::string wlabel, const int msec,
                      const double maxdepth=-1.0) const;
  
  /*!
   *  @brief  画像表示(表示範囲指定)
   *  @param  [in] wlabel  ラベル
   *  @param  [in] msec  タイムアウト値
   *  @param  [in] rect  表示範囲 ROI
   *  @param  [in] mag  表示倍率
   *  @retval UFV::OK  正常終了
   *  @note
   *    - 画像を表示する
   *  @attention  N/A
   */
  virtual int display(const std::string wlabel, const int msec,
                      const UFV::Rect &rect,
                      const double mag,
                      const double maxdepth=-1.0) const;

public:

  /*!
   *  @brief  画像ファイル書き込み
   *  @param  [in] name  ファイル名＋拡張子
   *  @note
   *    - Depth Mapに検出結果を描画した画像ファイルを書き込む
   *  @attention  N/A
   */
  virtual void writeImage(const std::string name="",
                          const double maxdepth=-1.0) const;
 
  /*!
   *  @brief  画像ファイル書き込み(表示範囲・倍率指定)
   *  @param  [in] rect  表示範囲 ROI
   *  @param  [in] mag  表示倍率
   *  @param  [in] name  ファイル名＋拡張子
   *  @note
   *    - Depth Mapに検出結果を描画した画像ファイルを書き込む
   *  @attention  N/A
   */
  virtual void writeImage(const UFV::Rect &rect,
                          const double mag=-1.0,
                          const std::string name="",
                          const double maxdepth=-1.0) const;

public:

  /*!
   *  @brief  検出した物体情報取得
   *  @param  [out] obs  検出した物体情報の格納先
   *  @note
   *    - obsにメンバ変数 m_obs_vecを代入する
   *  @attention
   *    - UFV::RangerImpl(または RangerImplEx)::_open で正しいFocalLengthを指定する必要がある
   */
  void getObstacle(std::vector<Obstacle> &obs) const
    {
      obs = m_obs_vec;
    };
  
  /*!
   *  @brief  DepthMapと検出結果のコピー
   *  @param  [out] bbox  コピー先
   *  @note
   *    - DepthMapを bboxにコピーし、検出結果を描画する
   *  @attention  N/A
   */
  void normalizeWithBoundingBox(ImageData<unsigned char> &bbox,
                                const double maxdepth) const;

  /*!
   *  @brief  検出結果描画
   *  @param  [out] bbox  検出結果を描画する画像
   *  @retval UFV::OK  正常終了
   *  @note
   *    - 検出結果を描画する
   *  @attention  N/A
   */
  virtual int drawBoundingBox(ImageData<unsigned char> &bbox) const;
  
  /*!
   *  @brief  検出結果描画(オフセット指定)
   *  @param  [out] bbox  検出結果を描画する画像
   *  @param  [in] offset  深度画像を基準としたオフセット値
   *  @retval UFV::OK  正常終了
   *  @note
   *    - オフセットを指定して検出結果を描画する。
   *      オフセット指定なしの検出結果描画関数は、深度画像と検出結果を描画する画像のサイズが
   *      同じであることが前提となっており、異なる場合はエラー復帰している。
   *      また、サイズが異なる場合にオフセット指定なしで描画しようとすると、
   *      検出結果の表示位置が深度画像と同じピクセル位置となる為、
   *      深度画像と輝度画像のサイズが異なる場合には、全く違う位置に検出結果が表示されてしまう。
   *      この問題は、本関数にて、深度画像を基準としたオフセット値を指定することで、回避できる。
   *  @attention  N/A
   *  @author 株式会社セック
   */
  virtual int drawBoundingBox(ImageData<unsigned char> &bbox,
                              const UFV::Point2Di &offset) const;


  /*!
  */
  virtual void printObstacles(FILE *fp = stdout) const;

private:

  //! 前景と背景の奥行き閾値 (mm)
  double m_zkey;
  
  //! ギャップの最小値 (mm) ※隣接する画素がこれより遠ければ別物体
  double m_min_gap;
  
  //! 物体の最小サイズ (pixel) ※これより小さい物体はノイズ
  int m_min_size;

protected:
  //! display()でラベルを描画するかどうか
  bool m_flg_drawlabel;
  
  //! 検出した物体の情報
  std::vector<Obstacle> m_obs_vec;

  //! ラベリング画像
  UFV::ImageData<short> m_labeled;
};

}

#endif
