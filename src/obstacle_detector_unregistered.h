// -*- C++ -*-
/*!
 * @file  obstacle_detector_unregistered.h
 * @brief emulated_srs::ObstacleDetectorUnregistered class definition
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2021  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __ROS_EMULATED_SRS_OBSTACLE_DETECTOR_UNREGISTERED_H__
#define __ROS_EMULATED_SRS_OBSTACLE_DETECTOR_UNREGISTERED_H__

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>

#include "UFV/ImageData.h"
#include <eSRS/ClassificationMap.h>

#include "intensity_map.h"

#include <emulated_srs/ClassifiedObstacle.h>
#include <emulated_srs/ClassifiedObstacleArray.h>

namespace emulated_srs
{

/*!
 * @class ObstacleDetectorUnregistered
 * @if jp
 *
 * @brief 物体検出 ROS ノード
 *
 * - PointCloud2 (organized) を subscribe
 * - 検出した障害物情報を publish
 *
 * @endif
 */
class ObstacleDetectorUnregistered
{
private:
  const int IMAGE_N_CHANNELS = 3;

public:
  //! Zキーの閾値距離 (mm)
  static const float TC_OPT_THRESHOLD_ZKEY; // 1500.0; // 2800.0;
  //! この距離より離れていれば別物体 (mm)
  static const float TC_OPT_THRESHOLD_GAP; // 100.0;
  //! 物体の最小サイズ (pixel)
  //const static int TC_OPT_MIN_BLOBSIZE = 2500; // 1280x720
  static const int TC_OPT_MIN_BLOBSIZE = 500; // VGA
  //! 距離画像での検出物体とYOLOの検出物体を同じ物体とみなす画像上での重なり率
  static const float TC_OPT_MIN_OVERLAP_RATE; // 0.8;

  static const std::string TC_OPT_MASKFILE; // "MASK.png"

public:
  ObstacleDetectorUnregistered(void);
  virtual ~ObstacleDetectorUnregistered(void) {};

private:
  virtual void initializeMap(const int width,
                             const int height);

  //!点群データをサブスクライブした際に呼び出されるコールバック関数
  void pc2Callback(
    const sensor_msgs::PointCloud2ConstPtr &pc2);

  //!点群データを2次元のマップデータに変換する
  bool convertPC2ToMapData(
    const sensor_msgs::PointCloud2ConstPtr &point_cloud2);

  //!点群データの内容を分析してデータ配列のフォーマット情報を抜き出す
  bool retrievePC2OffsetInfomation(
    const sensor_msgs::PointCloud2ConstPtr &pc2,
    unsigned long &x_offset,
    unsigned long &y_offset,
    unsigned long &z_offset);

  //!オフセット情報をもとに点群データから深度情報の2次元マップを作る
  bool createDepthMap(
    const sensor_msgs::PointCloud2ConstPtr &pc2,
    const unsigned long x_offset,
    const unsigned long y_offset,
    const unsigned long z_offset);

  void setMaskToMapData(void);

  virtual int execObstacleDetection(void);

  virtual void displayAll(void);
  
  void publishAll(
    const std::vector<eSRS::ObstacleClassified> &obstacle_classified,
    const int object_count);

  //!rvizでの表示用にMarkerとしてpublishする
  int publishMarkersMessage(
    const std::vector<eSRS::ObstacleClassified> &obs,
    const int nobs);

  //!障害物検知結果をpublishする
  int publishObstaclesMessage(
    const std::vector<eSRS::ObstacleClassified> &obs,
    const int nobs);

  //!障害物検出結果を可視化した画像をpublishする
  int publishImagesMessage(
    void);

protected:  
  //! the number of the obstacle detection executions
  unsigned int count_detection_;

  //! 障害物検知オブジェクト
  eSRS::ClassificationMap map_for_detection_;
  
  //!画像表示用データ
  UFV::ImageData<unsigned char> map_for_showing_depth_data_;

  //!距離画像におけるパラメータ
  float param_zkey_;
  float param_gap_;
  int param_min_size_;
  float param_min_overlap_;
  int param_use_mask_p_;
  int param_display_images_p_;
  int param_publish_images_p_;
  int param_publish_markers_p_;
  bool param_experimental_doublecheck_p_;

  bool flg_initialized_p_;

private:
  //!ROSノードハンドル
  ros::NodeHandle node_handle_;
  //!ROSサブスクライバ
  ros::Subscriber subscriber_;

  //!ROSパブリッシャ
  //!マーカーデータ用パブリッシャ
  ros::Publisher publisher_marker_;
  //!検知された障害物データ用パブリッシャ
  ros::Publisher publisher_obstacle_;
  //!距離画像データ用パブリッシャ
  image_transport::Publisher publisher_image_depth_;

  ros::Publisher publisher_exp_setup_; // experiment setup

  //! header of subscribed PC2
  std_msgs::Header header_pointcloud2_;

  //!検出結果をpublishしたときのタイムスタンプ
  ros::Time timestamp_detection_result_published_;

//!rosで画像を転送するためのモジュール
  image_transport::ImageTransport image_transport_;


  //マーカーデータ保存用配列
  std::vector<visualization_msgs::Marker> marker_;

}; //End of class ObstacleDetector

} // end of namespace emulated_srs

#endif // __ESRS_ROS_OBSTACLE_DETECTOR_H__
