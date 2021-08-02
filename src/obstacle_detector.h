// -*- C++ -*-
/*!
 * @file  obstacle_detector.h
 * @brief emulated_srs::ObstacleDetector class definition
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2020  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __ROS_EMULATED_SRS_OBSTACLE_DETECTOR_H__
#define __ROS_EMULATED_SRS_OBSTACLE_DETECTOR_H__

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>

#include <UFV/ImageData.h>
#include <eSRS/ClassificationMap.h>

#include <emulated_srs/Obstacle.h>

#include "intensity_map.h"

namespace emulated_srs
{

/*!
 * @class ObstacleDetector
 * @if jp
 *
 * @brief ROS node for obstaclle detection
 *
 * - Subscribe PointCloud2 (organized)
 * - Publish the data of detected obstacles
 *
 * @endif
 */
class ObstacleDetector
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
  static const std::string TC_OPT_LOGDIR; // "Data/"

  static const std::string LOGDIR_DEPTH; // "D/"
  static const std::string LOGDIR_DETECTION; // "R/"
  static const std::string LOGDIR_INTENSITY; // "I/"

public:
  ObstacleDetector(void);
  virtual ~ObstacleDetector(void) {};

protected:  
  virtual void initializeMap(const int width,
                             const int height,
                             const int point_step);

  virtual void displayAll(void);

  virtual void publishExpSetup(void);
  
  //!障害物検知結果をpublishする
  virtual int publishObstaclesMessage(
    const std::vector<eSRS::ObstacleClassified> &obs,
    const int nobs);

  virtual void setObstaclesMessage(
    const std::vector<eSRS::ObstacleClassified> &obs,
    const int nobs,
    std::vector<emulated_srs::Obstacle> &obsmsgary);

  virtual void publishAll(
    const std::vector<eSRS::ObstacleClassified> &obstacle_classified,
    const int object_count);

  //!rvizでの表示用にMarkerとしてpublishする
  virtual int publishMarkersMessage(
    const std::vector<eSRS::ObstacleClassified> &obs,
    const int nobs);

  //!障害物検出結果を可視化した画像をpublishする
  virtual int publishImagesMessage(void);

private:
  //!点群データをサブスクライブした際に呼び出されるコールバック関数
  void pc2Callback(
    const sensor_msgs::PointCloud2ConstPtr &pc2);

  int execObstacleDetection(void);

  //!点群データを2次元のマップデータに変換する
  bool convertPC2ToMapData(
    const sensor_msgs::PointCloud2ConstPtr &point_cloud2);

  //!点群データの内容を分析してデータ配列のフォーマット情報を抜き出す
  bool retrievePC2OffsetInfomation(
    const sensor_msgs::PointCloud2ConstPtr &pc2,
    unsigned long &x_offset,
    unsigned long &y_offset,
    unsigned long &z_offset,
    unsigned long &rgb_offset);

  //!オフセット情報をもとに点群データから深度情報の2次元マップを作る
  bool createDepthMapAndRGBMap(
    const sensor_msgs::PointCloud2ConstPtr &pc2,
    const unsigned long x_offset,
    const unsigned long y_offset,
    const unsigned long z_offset,
    const unsigned long rgba_offset);

  void reshapeMap(const int width,
                  const int height,
                  const int point_step);

  void maskMap(void);


public:
  std::string getLocalTimeString(ros::Time &rostime) const;

protected:
  //! param for the name of topic to subscribe to
  std::string param_name_topic_;
  
  //! true, if the subscribed PC2 has RGB information, 
  bool has_rgb_data_;
  
  //! number of the obstacle detection executions
  unsigned int count_detection_;
  
  //! instance for the obstacle detection
  eSRS::ClassificationMap map_for_detection_;
  
  //! instance for displaying RGB images
  emulated_srs::IntensityMapWithMask map_for_rgb_display_;

  //! depth and RGB image data for displaying
  UFV::ImageData<unsigned char> map_for_showing_depth_data_;
  UFV::ImageData<unsigned char> map_for_showing_rgb_data_;

  //! parameters for the obstacle detection
  float param_zkey_;
  float param_gap_;
  int param_min_size_;
  float param_min_overlap_;
  int param_use_mask_p_;
  int param_display_images_p_;
  int param_publish_images_p_;
  int param_publish_markers_p_;
  int param_save_images_p_;
  bool param_experimental_doublecheck_p_;

  //! parameters on experimetal setup
  std::string param_name_sensor_;
  std::string param_fname_mask_;
  std::string param_dname_log_;
  //std::string param_fname_region_;

  //! set true after the 1st subscription of PC2
  bool flg_initialized_p_;

  std::string basename_to_save_images_; //!< updated for every subscription

protected:
  //! ROS node handle
  ros::NodeHandle node_handle_;
  
private:
  //! ROS subscriber
  ros::Subscriber subscriber_;

  //! ROS publishers
  ros::Publisher publisher_marker_;
  image_transport::Publisher publisher_image_depth_;
  image_transport::Publisher publisher_image_rgb_;

protected:
  ros::Publisher publisher_obstacle_;
  ros::Publisher publisher_exp_setup_; // experiment setup

private:
  //! header of subscribed PC2
  std_msgs::Header header_pointcloud2_;

  //! timestamp when publishing obstacle data
  ros::Time timestamp_detection_result_published_;

  //! ROS image transporter
  image_transport::ImageTransport image_transport_;

}; //End of class ObstacleDetector

} // end of namespace emulated_srs

#endif // __ESRS_ROS_OBSTACLE_DETECTOR_H__
