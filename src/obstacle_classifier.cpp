// -*- C++ -*-
/**
 * @file  obstacle_classifier.cpp
 * @brief A ROS node for obstacle detection with classification
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 * 
 * Copyright (C) 2019  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <ros/ros.h>

#include "obstacle_classifier.h"

/*!
 *  @brief main function
*/
int main(int argc, char** argv)
{
  ros::init (argc, argv, "obstacle_classifier");

  emulated_srs::ObstacleClassifier obstacle_classifier;

  ros::spin();

  return 0;
}

void emulated_srs::ObstacleClassifier::initializeMap(
  const int width,
  const int height)
{
  this->emulated_srs::ObstacleDetector::initializeMap(width, height, 32);

  map_for_classification_.reshape(width, height);
  ROS_INFO_ONCE("YoloMapMask");

  return;
}

/*!
 * @if jp
 * @brief RGB画像データをAIで分析、分類を行い、その情報を元に物体分類画像を作成
 * @retval AIで確認できたオブジェクトの数 
 * @endif
*/
int emulated_srs::ObstacleClassifier::execObstacleDetection(void)
{
  int object_count(0);

  //UFV::ImageData<unsigned char> mimg =*(map_for_rgb_display_.getImageData<unsigned char>());
  //mimg.display("CP", -1);
  //map_for_classification_.setData<unsigned char>(mimg);
  map_for_classification_.setData<unsigned char>(*(map_for_rgb_display_.getImageData<unsigned char>()));

  //AIによる物体分類を行う
  map_for_classification_.detect();
  count_detection_++;

  //分類を行ったデータラベルの取得
  std::vector<eSRS::YoloObject> yobjs;
  map_for_classification_.getLabeledObject(yobjs);

  //物体検知とオブジェクト判断処理の合成をする(物体名と枠のオーバーライド)
  object_count = map_for_detection_.detect(yobjs,
                                           param_experimental_doublecheck_p_);

  return object_count;
}

void emulated_srs::ObstacleClassifier::displayAll(void)
{
  if(!(param_display_images_p_ || param_publish_images_p_)) return;
  
  // 輝度画像を表示
  //map_for_showing_rgb_data_ = *(map_for_rgb_display_.getImageData<unsigned char>());
  map_for_showing_rgb_data_ = *(map_for_classification_.getImageData<unsigned char>());

  map_for_detection_.drawObstacleRegion(map_for_showing_depth_data_);
  map_for_classification_.drawBoundingBox(map_for_showing_rgb_data_);
  //map_for_detection_.drawBoundingBox(map_for_showing_rgb_data_);

  if (param_display_images_p_)
  {
    map_for_showing_rgb_data_.display("Yolo", 10);
    map_for_showing_depth_data_.display("Depth", 10);
  }

  return;
  
}
