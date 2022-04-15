// -*- C++ -*-
/**
 * @file  obstacle_detector.cpp
 * @brief A class of a ROS node for obstacle detection with classification
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

#include <unistd.h>
#include <string>
#include <vector>
#include <limits>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/core.hpp>

#include <emulated_srs/Obstacle.h>
#include <emulated_srs/ExpSetup.h>

#include "UFV/utils.h"
#include "obstacle_detector.h"

const float emulated_srs::ObstacleDetector::TC_OPT_THRESHOLD_ZKEY = 1500.0;
const float emulated_srs::ObstacleDetector::TC_OPT_THRESHOLD_GAP = 100.0;
const float emulated_srs::ObstacleDetector::TC_OPT_MIN_OVERLAP_RATE = 0.8;
const std::string emulated_srs::ObstacleDetector::TC_OPT_MASKFILE = "MASK.png";
const std::string emulated_srs::ObstacleDetector::TC_OPT_LOGDIR = "Data/";

const std::string emulated_srs::ObstacleDetector::LOGDIR_DEPTH = "D/";
const std::string emulated_srs::ObstacleDetector::LOGDIR_DETECTION = "R/";
const std::string emulated_srs::ObstacleDetector::LOGDIR_INTENSITY = "I/";

/*!
int main(int argc, char** argv)
{
  ros::init (argc, argv, "obstacle_detector");

  emulated_srs::ObstacleDetector obstacle_detector;

  ros::spin();

  return 0;
}
*/

emulated_srs::ObstacleDetector::ObstacleDetector(void)
    :
    param_name_topic_("/camera/depth_registered/points"),
    has_rgb_data_(true),
    count_detection_(0),
    param_zkey_(TC_OPT_THRESHOLD_ZKEY),
    param_gap_(TC_OPT_THRESHOLD_GAP),
    param_min_size_(TC_OPT_MIN_BLOBSIZE),
    param_min_overlap_(TC_OPT_MIN_OVERLAP_RATE),
    param_use_mask_p_(0),
    param_display_images_p_(0),
    param_publish_images_p_(0),
    param_publish_markers_p_(0),
    param_save_images_p_(0),
    param_experimental_doublecheck_p_(false),
    param_name_sensor_(""),
    param_fname_mask_(TC_OPT_MASKFILE),
    param_dname_log_(TC_OPT_LOGDIR),
    flg_initialized_p_(false),
    basename_to_save_images_("IMG.png"),
    node_handle_("~"),
    image_transport_(node_handle_)
{
  int param_doublecheck=0;
  
  // Get from .launch
  node_handle_.getParam("topic_name", param_name_topic_);
  node_handle_.getParam("sensor_name", param_name_sensor_);
  
  node_handle_.getParam("zkey", param_zkey_);
  node_handle_.getParam("min_gap_of_occluding_boundary", param_gap_);
  node_handle_.getParam("min_pixels_as_object", param_min_size_);
  node_handle_.getParam("min_overlap_rate", param_min_overlap_);
  node_handle_.getParam("use_mask_p", param_use_mask_p_);
  node_handle_.getParam("display_images_p", param_display_images_p_);
  node_handle_.getParam("publish_images_p", param_publish_images_p_);
  node_handle_.getParam("publish_markers_p", param_publish_markers_p_);
  node_handle_.getParam("save_images_p", param_save_images_p_);
  node_handle_.getParam("experimental_doublecheck_p", param_doublecheck);

  node_handle_.getParam("filename_mask", param_fname_mask_);
  node_handle_.getParam("dirname_log", param_dname_log_);

  // print them on the console for confirmation
  ROS_INFO("topic to subscribe to: %s", param_name_topic_.c_str());
  ROS_INFO("sensor: %s", param_name_sensor_.c_str());

  ROS_INFO("zkey: %f", param_zkey_);
  ROS_INFO("min_gap_of_occluding_boundary: %f", param_gap_);
  ROS_INFO("min_pixels_as_object: %d", param_min_size_);
  ROS_INFO("min_overlap_rate: %f", param_min_overlap_);
  ROS_INFO("use_mask_p: %d", param_use_mask_p_);
  ROS_INFO("filename_mask: %s", param_fname_mask_.c_str());
  ROS_INFO("dirname_log: %s", param_dname_log_.c_str());
  ROS_INFO("display_images_p: %d", param_display_images_p_);
  ROS_INFO("publish_images_p: %d", param_publish_images_p_);
  ROS_INFO("publish_markers_p: %d", param_publish_markers_p_);
  ROS_INFO("XXsave_images_p: %d", param_save_images_p_);
  ROS_INFO("experimental_doublecheck_p: %d", param_doublecheck);
  param_experimental_doublecheck_p_ = param_doublecheck;

  // publishers and subscribers
  publisher_marker_ = node_handle_.advertise<visualization_msgs::Marker>(
      "visualization_marker", 1000);

  publisher_obstacle_ = node_handle_.advertise<emulated_srs::Obstacle>(
      "obstacle", 1000);

  publisher_image_depth_ = image_transport_.advertise(
      "depth/image_raw", 10);

  publisher_image_rgb_ = image_transport_.advertise(
      "color/image_raw", 10);

  publisher_exp_setup_ =
    node_handle_.advertise<emulated_srs::ExpSetup>(
      "setup_experiment", 1, true); // enable latch

  ROS_INFO("publishers: OK");

  //timestamp_pointcloud2_subscribed_ = ros::Time::now();

  subscriber_ = node_handle_.subscribe(param_name_topic_, 10,
                                       &emulated_srs::ObstacleDetector::pc2Callback, this);

  ROS_INFO("subscriber: OK");

}


/*!
 * @if jp
 *
 * @brief callback function at sbscribing PC2 data
 *
 * - Convert the PC2 data to depth and RGB images
 * - Exec obstacle detection
 * - Publish objstacle information as ClassifiledObstacle
 *
 * @param[in] pc2 Organized PC2 data
 * @return
 *
 * @else
 * @endif
 */
void emulated_srs::ObstacleDetector::pc2Callback(
    const sensor_msgs::PointCloud2ConstPtr &pc2)
{
  bool bret;
  int object_count;
  std::vector<eSRS::ObstacleClassified> obstacle_classified;

  ROS_INFO_ONCE("PC2 received: %d %d", pc2->width, pc2->height);

  if(pc2->height <= 1 || pc2->width <= 1)
  {
    ROS_WARN("An unorganized PC2 has subscribed: %d x %d", 
             pc2->width, pc2->height);
    return;
  }

  if(pc2->point_step != 32 && pc2->point_step != 16)
  {
    ROS_WARN("Unsupported point_step: %d", 
             pc2->point_step);
    return;
  }

  // Mapオブジェクトの初期化と初期設定を行う。
  // このコールバック関数で初めてサブスクライブした
  // 画像のサイズがわかるため、このタイミングで初期化が必要
  if (flg_initialized_p_ == false)
  {
    initializeMap(pc2->width, pc2->height, pc2->point_step);
    ROS_INFO_ONCE("initialize completed: %d x %d (%d)",
                  pc2->width, pc2->height, pc2->point_step);
  }

  //pointcloud2からデータを抜き取り画像データ深度画像とrgb画像を作る。
  bret = convertPC2ToMapData(pc2);
  if (bret == false)
  {
    //失敗した原因については各関数内でROS_WARN(),ROS_ERROR()で出力している
    //ので、ここでは出力しない
    return;
  }
  ROS_INFO_ONCE("data conversion completed");

  // timestamp and frame_id
  header_pointcloud2_ = pc2->header;
  basename_to_save_images_ = getLocalTimeString(header_pointcloud2_.stamp) + ".png";

  // apply masking to the depth MAP image
  maskMap();
  ROS_INFO_ONCE("masking completed");

  // exec obect detection
  object_count = execObstacleDetection();
  //map_for_detection_.display("Det", -1);

  ROS_INFO_ONCE("obstacle detection completed");

  // make obstacle information for publishing
  map_for_detection_.getObstacleClassified(obstacle_classified);

  ROS_INFO_ONCE("list prepared");

  timestamp_detection_result_published_ = ros::Time::now();

  // exec displaying and saving
  displayAll();

  // exec publishing
  publishAll(obstacle_classified, object_count);

  ROS_INFO_ONCE("published");

  return;
}

/*!
 * @if jp
 *
 * @brief 深度画像、rgb画像用のオブジェクトのインスタンス化と初期設定を行う
 * @param[in] width 画像の幅
 * @param[in] height 画像の高さ
 * @note ROSの機能により、.launchファイルでparam_xxの値は変更できます。
 *
 * @endif
 */
void emulated_srs::ObstacleDetector::initializeMap(
  const int width,
  const int height,
  const int point_step)
{
  this->reshapeMap(width, height, point_step);

  map_for_detection_.setZkey(param_zkey_);                  // mm
  map_for_detection_.setMinGap(param_gap_);                 // mm
  map_for_detection_.setMinSize(param_min_size_);           // pixel
  map_for_detection_.setMinOverlapRate(param_min_overlap_); // 0..1
  map_for_detection_.setDrawLabel(false); // 0..1
  if (param_use_mask_p_ != 0)
  {
    //map_for_detection_.setMaskImage(TC_OPT_MASKFILE);
    map_for_detection_.setMaskImage(param_fname_mask_);
  }

  flg_initialized_p_ = true;

  return;
}

void emulated_srs::ObstacleDetector::reshapeMap(
  const int width,
  const int height,
  const int point_step)
{
  if(map_for_detection_.width() == width &&
     map_for_detection_.height() == height &&
     (has_rgb_data_ && (point_step == 32)))
  {
    return;
  }

  has_rgb_data_ = false;
  
  if(point_step == 32) // XYZRGB
  {
    map_for_rgb_display_.reshape(width, height);
    ROS_INFO_ONCE("IntensityMapWithMask");
    has_rgb_data_ = true;
  }

  map_for_detection_.reshape(width, height);
  ROS_INFO_ONCE("ClassificationMap");

  map_for_showing_depth_data_.reshape(width, height, IMAGE_N_CHANNELS);  
  if(point_step == 32) // XYZRGB
  {
    map_for_showing_rgb_data_.reshape(width, height, IMAGE_N_CHANNELS);
  }
  ROS_INFO_ONCE("reshape");
  
  return;
}

std::string
emulated_srs::ObstacleDetector::getLocalTimeString(ros::Time &rostime) const
{
  UFV::Time tm(rostime.sec, rostime.nsec/1000);
  UFV::LocalTime ltm = getLocalTime(tm);

  return(UFV::getLocalTimeString(ltm));
}

/*!
 * @if jp
 *
 * @brief 点群データから深度画像と輝度画像データを抜き出す
 * @param[in] point_cloud2 pointcloud2データ
 * @return 処理の成否
 * @retval false 処理失敗
 *
 * @endif
 */
bool emulated_srs::ObstacleDetector::convertPC2ToMapData(
    const sensor_msgs::PointCloud2ConstPtr &point_cloud2)
{
  unsigned long x_offset;
  unsigned long y_offset;
  unsigned long z_offset;
  unsigned long rgb_offset;
  bool result;

  // reshape Maps, if necessary
  reshapeMap(point_cloud2->width, point_cloud2->height,
             point_cloud2->point_step);

  //pointcloud2メッセージからx,y,z,rgbの情報が保存されている配列のオフセット情報を取り出す
  result = retrievePC2OffsetInfomation(point_cloud2,
                                       x_offset, y_offset, z_offset,
                                       rgb_offset);
  ROS_INFO_ONCE("offsets: %lu %lu %lu, %lu",
                x_offset, y_offset, z_offset, rgb_offset);
  if(result == false)
  {
    return result;
  }

  //オフセット情報をもとに点群データから深度情報の2次元マップを作る （map_for_detection_に値が入る）
  result = createDepthMapAndRGBMap(point_cloud2,
                                   x_offset, y_offset, z_offset,
                                   rgb_offset);
  if (result == false)
  {
    return result;
  }

  return true;
}

/*!
 * @if jp
 * @brief 処理画像のマスク処理を行う
 * @endif
*/
void emulated_srs::ObstacleDetector::maskMap()
{
  bool ret(false);

  ret = map_for_detection_.hasMaskImage();
  if (ret == true)
  {
    map_for_detection_.mask();
  }

  if(has_rgb_data_)
  {
    //ret = map_for_classification_.hasMaskImage();
    ret = map_for_rgb_display_.hasMaskImage();
    if (ret == true)
    {
      //map_for_classification_.mask();
    map_for_rgb_display_.mask();
    }
  }

  return;
}

/*!
 * @if jp
 * @brief pointcloud2データを解析する。想定したデータフォーマットでは無い場合Falseを返す
 * @param[in] pc2 サブスクライブしたpointcloud2メッセージデータ
 * @param[out] xoffset
 * @param[out] yoffset
 * @param[out] zoffset
 * @param[out] rgboffset
 * @retval false 解析できなかった（想定していたフォーマットでは無い）
 * @retval true 解析完了
 * @note
 * http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
 * ex) realsense ros D435
 * <PRE>
 * height: 480  #画像高さ
 * width: 640   #画像幅
 * fields:      #PointCloud2.data[]のフォーマット情報
 *   - 
 *     name: "x"
 *     offset: 0
 *     datatype: 7  #=> FLOAT32
 *     count: 1
 *   - 
 *     name: "y"
 *     offset: 4
 *     datatype: 7
 *     count: 1
 *   - 
 *     name: "z"
 *     offset: 8
 *     datatype: 7
 *     count: 1
 *   - 
 *     name: "rgb"
 *     offset: 16
 *     datatype: 7      #=>FLOAT32となっているが実際はUINT32でRGBA情報で飛んできている（Aは透明度）
 *     count: 1
 * is_bigendian: False  #エンディアン
 * point_step: 32       #1つの情報のバイトサイズ
 * row_step: 20480      #1行あたりの情報のバイトサイズ -- row_step x height = 全情報
 *                                                     -- point_step x width = row_step
 * </PRE>
 * |x   |y   |z   |na  |rgba|other info  |  32バイトというフォーマットになっている
 * D435ではこのようになっているが、他のカメラでもこの32バイトフォーマットが一般的でother infoの場所にデータ信頼性や付属情報が載る 
 * @endif
 */
bool emulated_srs::ObstacleDetector::retrievePC2OffsetInfomation(
    const sensor_msgs::PointCloud2ConstPtr &pc2,
    unsigned long &x_offset,
    unsigned long &y_offset,
    unsigned long &z_offset,
    unsigned long &rgb_offset)
{
  int fsize;
  std::string str;

  fsize = pc2->fields.size();

  x_offset = 0xFFFFFFFF;
  y_offset = 0xFFFFFFFF;
  z_offset = 0xFFFFFFFF;
  rgb_offset = 0xFFFFFFFF;

  //x,y,zの深度データのオフセット情報を取得
  for(int i = 0; i < fsize; i++)
  {
    str = pc2->fields[i].name;
    if(str == "x")
    {
      x_offset = pc2->fields[i].offset;
    }
    else if(str == "y")
    {
      y_offset = pc2->fields[i].offset;
    }
    else if(str == "z")
    {
      z_offset = pc2->fields[i].offset;
    }
    else if(str == "rgb")
    {
      rgb_offset = pc2->fields[i].offset;
    }
  }

  //check
  if ((x_offset == 0xFFFFFFFF) ||
      (y_offset == 0xFFFFFFFF) ||
      (z_offset == 0xFFFFFFFF))
  {
    ROS_ERROR("Unsupported PC2. No offset data.");
    return false;
  }

  return true;
}

/*!
 * @if jp
 *
 * @brief pointcloud2データをeSRSライブラリで使用する配列に整列する
 *
 * pointcloud2のfloat(x,y,z)の深度データを2次元float配列データに変換する
 *
 * @param[in] pc2 pointcloud2データ。参照のみ、変更はしないこと。
 * @param[in] x_offset "x軸方向の距離データ"の配列内オフセット値
 * @param[in] y_offset "y軸方向の距離データ"の配列内オフセット値
 * @param[in] z_offset "z軸方向の距離データ"の配列内オフセット値
 * @param[in] rgb_offset "rgbデータ"の配列内オフセット値
 *
 * @endif
 */
bool emulated_srs::ObstacleDetector::createDepthMapAndRGBMap(
    const sensor_msgs::PointCloud2ConstPtr &pc2,
    unsigned long x_offset,
    unsigned long y_offset,
    unsigned long z_offset,
    unsigned long rgba_offset)
{
  if (map_for_detection_.width() == 0)
  {
    ROS_ERROR("program fail -- read code %s %d", __FILE__, __LINE__);
    return false;
  }

  const unsigned char *pc2data;
  float *x_dep;
  float *y_dep;
  float *z_dep;
  unsigned char *i_map;
  int point_step;
  int pc2_data_size;
  int map_idx;

  //mapの先頭ポインタを取得。配列のサイズはイニシャライズ時に確定している
  x_dep = map_for_detection_.xdata();
  y_dep = map_for_detection_.ydata();
  z_dep = map_for_detection_.data();

  pc2data = pc2->data.data();                            //pointcloud2にある生データ配列の先頭ポインタ
  point_step = pc2->point_step;                          //pointcloud2にある生データ配列の1データブロックあたりのサイズ
  pc2_data_size = pc2->width * pc2->height * point_step; //pointcloud2にある生データ配列のサイズ

  if(point_step == 32) // point_step:32 -> xyzrgb, 16->xyz
  {
    //i_map = map_for_classification_.getData<unsigned char>();
    i_map = map_for_rgb_display_.getData<unsigned char>();
  }

  /*!
   * * point_step: 32
   *   - berore
   *       |x1|y1|z1|n|rgba1|nn|x2|y2|z2|n|rgba2|nn|x3|y3|z3|・・・
   *   - after
   *       |x1|x2|x3|・・・
   *       |y1|y2|y3|・・・
   *       |z1|z2|z3|・・・
   *       |r1|g1|b1|r2|g2|b2|r3|g3|b3|・・・
   *
   * * point_step: 16
   *   - berore
   *       |x1|y1|z1|nx2|y2|z2|n|x3|y3|z3|・・・
   *   - after
   *       |x1|x2|x3|・・・
   *       |y1|y2|y3|・・・
   *       |z1|z2|z3|・・・
   */
  map_idx = 0;
  for (int i = 0; i < pc2_data_size; i += point_step)
  {
    //メッセージのuint8配列データをfloatに変換してコピー
    x_dep[map_idx] = *((float *)(&pc2data[i + x_offset]));
    y_dep[map_idx] = *((float *)(&pc2data[i + y_offset]));
    z_dep[map_idx] = *((float *)(&pc2data[i + z_offset]));

    //x,y,zの値で評価するのはzだけで良い
    //if (z_dep[map_idx] == std::numeric_limits<float>::quiet_NaN())
    if (z_dep[map_idx] != z_dep[map_idx]) // NaN == NaN is false
    {
      x_dep[map_idx] = 0;
      y_dep[map_idx] = 0;
      z_dep[map_idx] = eSRS::DepthMap::PIXELVALUE_NODATA;
    }
    else
    {
      //座標系の単位を合わせる(m -> mm)
      x_dep[map_idx] *= 1000;
      y_dep[map_idx] *= 1000;
      z_dep[map_idx] *= 1000;
    }

    if(point_step == 32) // point_step:32 -> xyzrgb, 16->xyz
    {
      //rgba処理(aは処理しない)
      i_map[(map_idx * 3)] = pc2data[(i + rgba_offset)];         //r
      i_map[(map_idx * 3) + 1] = pc2data[(i + rgba_offset) + 1]; //g
      i_map[(map_idx * 3) + 2] = pc2data[(i + rgba_offset) + 2]; //b
    }

    map_idx++;
  } //loopend

  return true;
}

/*!
 * @if jp
 * @brief 障害物検出を実施する
 * @retval 検出した障害物の数 
 * @endif
 */
int
emulated_srs::ObstacleDetector::execObstacleDetection(void)
{
  int object_count = 0;
  object_count = map_for_detection_.detect();
  count_detection_++;
  ROS_INFO("Detector: %d", count_detection_);
  
  return object_count;
}

/*!
 * @if jp
 * @brief 各種データのパブリッシュを行う
 * @param[in] obstacle_classified 検知した障害物の情報
 * @param[in] object_count 検知した障害物の数
 * @note 物体検知、分類のデータは必ずパブリッシュする。
 *       マーカーや画像データをパブリッシュするかどうかは起動時のlaunchに従う。
 * @endif
*/
void
emulated_srs::ObstacleDetector::publishAll(
  const std::vector<eSRS::ObstacleClassified> &obstacle_classified,
  const int object_count)
{
  if(count_detection_ == 1)
  {
    publishExpSetup();
  }
    
  publishObstaclesMessage(obstacle_classified, object_count);

  if (param_publish_markers_p_)
  {
    publishMarkersMessage(obstacle_classified, object_count);
  }
  if(param_publish_images_p_)
  {
    publishImagesMessage();
  }
  return;
}

void emulated_srs::ObstacleDetector::publishExpSetup(void)
{
  // publish the experimental setup as a latch topic
  emulated_srs::ExpSetup exp_setup;
  exp_setup.name_sensor = param_name_sensor_;
  exp_setup.dist_testpiece = -1.0;
  exp_setup.fname_mask = param_use_mask_p_ ? param_fname_mask_ : "";
  //exp_setup.fname_region = param_fname_region_;
  exp_setup.fname_region = "";
  exp_setup.param_zkey = param_zkey_;
  exp_setup.param_min_gap = param_gap_;
  exp_setup.param_min_size = param_min_size_;

  publisher_exp_setup_.publish(exp_setup);

  return;
}

void emulated_srs::ObstacleDetector::displayAll(void)
{
  if(!(param_display_images_p_ ||
       param_publish_images_p_ ||
       param_save_images_p_))
    return;

  // Copy the depth image with rtection results for display
  map_for_detection_.normalize(map_for_showing_depth_data_);

  if(has_rgb_data_)
  {
    // Copy the current RGB image.
    map_for_showing_rgb_data_ = *(map_for_rgb_display_.getImageData<unsigned char>());
  }

  // Overwrite the depth image with the obstacle reasons.
  map_for_detection_.drawObstacleRegionWithLabel(map_for_showing_depth_data_);

  // Draw the bounding boxes for the obstacles on the RGB image.
  /*
  if(has_rgb_data_)
  {
    map_for_detection_.drawBoundingBox(map_for_showing_rgb_data_);
  }
  */

  if (param_display_images_p_)
  {
    // Display the images.
    if(has_rgb_data_)
    {
      map_for_showing_rgb_data_.display("RGB", -1);
    }
    map_for_detection_.display("Depth",-1);
    map_for_showing_depth_data_.display("Detection", 10);
  }

  if(param_save_images_p_)
  {
    // save the depth image
    map_for_detection_.setWriteImage(true,
                                     param_dname_log_ + LOGDIR_DEPTH);
    map_for_detection_.writeImage(basename_to_save_images_);

    // save the depth image with detection result
    map_for_showing_depth_data_.setWriteImage(true,
                                    param_dname_log_ + LOGDIR_DETECTION);
    map_for_showing_depth_data_.writeImage(basename_to_save_images_);

    if(has_rgb_data_)
    {
      // save the rgb image
      map_for_showing_rgb_data_.setWriteImage(true, 
                                    param_dname_log_ + LOGDIR_INTENSITY);
      map_for_showing_rgb_data_.writeImage(basename_to_save_images_);
    }
    
    ROS_INFO_ONCE("SAVED: %s", (param_dname_log_ + LOGDIR_DEPTH + basename_to_save_images_).c_str());
  }

  return;
  
}

/*!
 * @if jp
 * 障害物検知、分類済み画像データをpublishする
 * @endif
 */
int emulated_srs::ObstacleDetector::publishImagesMessage(void)
{
  sensor_msgs::ImagePtr msg;

  if(has_rgb_data_)
  {
    cv::Size ysize(map_for_showing_rgb_data_.width(),
                   map_for_showing_rgb_data_.height());
    cv::Mat yoloimg(ysize, CV_8UC3, map_for_showing_rgb_data_.data());

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", yoloimg).toImageMsg();

    publisher_image_rgb_.publish(msg);
  }

  cv::Size dsize(map_for_showing_depth_data_.width(),
                 map_for_showing_depth_data_.height());
  cv::Mat detimg(dsize, CV_8UC3, map_for_showing_depth_data_.data());

  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detimg).toImageMsg();
  publisher_image_depth_.publish(msg);

  return UFV::OK;
}

/*!
 * @brief Publish obstacle detection results.
 *          - 3D coord-sys: Right-handed, sensor orig, optic-axis:z, down:y
 *          - 2D coord-sys: upper-left orig, right:x, down:y
 * @param[in] obstacle_classified      detected obstacles
 * @param[in] object_count             number of the obstacles
 * @note
 * rostopic echo -n 1 /emulated_srs/obstacle
 * <PRE>
 * header: 
 *   seq: 0
 *   stamp:                               # time of PC2 data acquired
 *     secs: 1626834032
 *     nsecs:  49269199
 *   frame_id: "camera_color_optical_frame"
 * n: 0                                   # obstacle number
 * position_3D:                           # at front, upper, left vertex
 *   x: -0.060076452791690826             #    of the bounding cuboid
 *   y: -0.02606579288840294
 *   z: 1.2140001058578491
 * dimensions_3D:                         # of the bounding cuboid
 *   x: 0.23947076499462128
 *   y: 0.6017187237739563
 *   z: 0.28299999237060547
 * centroid_3D:                           # of the points
 *   x: 0.060652490705251694              # NOT of the bounding cuboid
 *   y: 0.197583869099617
 *   z: 1.2704710960388184
 * position_2D:                           # of the bounding box
 *   x: 301.0
 *   y: 228.0
 * dimensions_2D:                         # of the bounding box
 *   x: 115.0
 *   y: 251.0
 * centroid_2D:                           # of the points
 *   x: 60.65249252319336
 *   y: 197.5838623046875
 * n_points: 19902                        # num of the points
 * n_points_within: 0                     #        within the correct region
 * filename_saved: "20210721_112032_49269199.png"
 * type_class: "unknown"
 * confidence_class: 0.0
 * ---
 * </PRE>
 */

void emulated_srs::ObstacleDetector::setObstaclesMessage(
  const std::vector<eSRS::ObstacleClassified> &obstacle_classified,
  const int object_count,
  std::vector<emulated_srs::Obstacle> &obsmsgary)
{
  //std::string ifname = getLocalTimeString(header_pointcloud2_.stamp) + ".png";

  if (object_count <= 0)
  {
    emulated_srs::Obstacle obsmsg;

    obsmsg.header = header_pointcloud2_;
    obsmsg.header.seq = count_detection_;

    obsmsg.n = -1;

    obsmsg.filename_saved = basename_to_save_images_;

    obsmsg.type_class = "none";
    obsmsg.confidence_class = 0.0;

    obsmsg.position_3D.x = 0.0;
    obsmsg.position_3D.y = 0.0;
    obsmsg.position_3D.z = 0.0;

    obsmsg.dimensions_3D.x = 0.0;
    obsmsg.dimensions_3D.y = 0.0;
    obsmsg.dimensions_3D.z = 0.0;

    obsmsg.centroid_3D.x = 0.0;
    obsmsg.centroid_3D.y = 0.0;
    obsmsg.centroid_3D.z = 0.0;

    obsmsg.position_2D.x = 0.0;
    obsmsg.position_2D.y = 0.0;

    obsmsg.dimensions_2D.x = 0.0;
    obsmsg.dimensions_2D.y = 0.0;

    obsmsg.centroid_2D.x = 0.0;
    obsmsg.centroid_2D.y = 0.0;

    obsmsg.n_points = 0;
    obsmsg.n_points_within = 0;

    //publisher_obstacle_.publish(obsmsg);
    obsmsgary.push_back(obsmsg);
  }
  else
  {
    for (int i = 0; i < object_count; i++)
    {
      emulated_srs::Obstacle obsmsg;

      obsmsg.header = header_pointcloud2_;
      obsmsg.header.seq = 0;

      obsmsg.filename_saved = basename_to_save_images_;

      obsmsg.n = obstacle_classified[i].n;
      obsmsg.type_class = obstacle_classified[i].classstr;
      obsmsg.confidence_class = obstacle_classified[i].conf;

      obsmsg.position_3D.x = obstacle_classified[i].bvol.x / 1000.0;
      obsmsg.position_3D.y = obstacle_classified[i].bvol.y / 1000.0;
      obsmsg.position_3D.z = obstacle_classified[i].bvol.z / 1000.0;

      obsmsg.dimensions_3D.x = obstacle_classified[i].bvol.width / 1000.0;
      obsmsg.dimensions_3D.y = obstacle_classified[i].bvol.height / 1000.0;
      obsmsg.dimensions_3D.z = obstacle_classified[i].bvol.depth / 1000.0;

      obsmsg.centroid_3D.x = obstacle_classified[i].grv.x / 1000.0;
      obsmsg.centroid_3D.y = obstacle_classified[i].grv.y / 1000.0;
      obsmsg.centroid_3D.z = obstacle_classified[i].grv.z / 1000.0;

      obsmsg.position_2D.x = obstacle_classified[i].bbox.x;
      obsmsg.position_2D.y = obstacle_classified[i].bbox.y;

      obsmsg.dimensions_2D.x = obstacle_classified[i].bbox.width;
      obsmsg.dimensions_2D.y = obstacle_classified[i].bbox.height;

      obsmsg.centroid_2D.x = obstacle_classified[i].grv.x;
      obsmsg.centroid_2D.y = obstacle_classified[i].grv.y;

      obsmsg.n_points = obstacle_classified[i].size;
      obsmsg.n_points_within = 0;

      //publisher_obstacle_.publish(obsmsg);
      obsmsgary.push_back(obsmsg);
    }
  }

  return;
}

int emulated_srs::ObstacleDetector::publishObstaclesMessage(
  const std::vector<eSRS::ObstacleClassified> &obstacle_classified,
  const int object_count)
{
  std::vector<emulated_srs::Obstacle> obsmsgary;

  setObstaclesMessage(obstacle_classified, object_count, obsmsgary);
  
  //! if object_count==0, publish an  empty obstacle
  publisher_obstacle_.publish(obsmsgary[0]);
  for(int i=1; i < object_count; i++)
  {
    publisher_obstacle_.publish(obsmsgary[i]);
  }
  //publisher_obstacle_.publish(obsmsgary);

  return UFV::OK;
}


/**
 * @if jp
 * @brief  分類、検知された障害物のデータをrvizで3D表示するための
 *         visualization_msgs/Marker.msgのフォーマットでパブリッシュする
 * @param[in] obstacle_classified 検知した障害物の情報
 * @param[in] object_count 検知した障害物の数
 * @endif
*/
int emulated_srs::ObstacleDetector::publishMarkersMessage(
    const std::vector<eSRS::ObstacleClassified> &obs,
    const int nobs)
{
  visualization_msgs::Marker marker;

  marker.header = header_pointcloud2_;
  marker.header.seq = count_detection_;

  for (int i = 0; i < nobs; i++)
  {
    marker.ns = "basic_shapse";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    //marker.header.stamp = timestamp_subscrib_pointcloud2_;
    //marker.header.stamp = timestamp_detection_result_published_;

    marker.pose.position.z = (obs[i].bvol.z + obs[i].bvol.depth / 2) / 1000.0;
    marker.pose.position.x = (obs[i].bvol.x + obs[i].bvol.width / 2) / 1000.0;
    marker.pose.position.y = (obs[i].bvol.y + obs[i].bvol.height / 2) / 1000.0;
    //marker.pose.position.x = (obs[i].bvol.z + obs[i].bvol.depth / 2) / 1000.0;
    //marker.pose.position.y = -(obs[i].bvol.x + obs[i].bvol.width / 2) / 1000.0;
    //marker.pose.position.z = -(obs[i].bvol.y + obs[i].bvol.height / 2) / 1000.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.z = obs[i].bvol.depth / 1000.0;
    marker.scale.x = obs[i].bvol.width / 1000.0;
    marker.scale.y = obs[i].bvol.height / 1000.0;
    //marker.scale.x = obs[i].bvol.depth / 1000.0;
    //marker.scale.y = obs[i].bvol.width / 1000.0;
    //marker.scale.z = obs[i].bvol.height / 1000.0;

    UFV::Color ucolor = obs[i].color;
    marker.color.r = ucolor.r / 255.0;
    marker.color.g = ucolor.g / 255.0;
    marker.color.b = ucolor.b / 255.0;
    marker.color.a = 0.5;

    publisher_marker_.publish(marker);

  }

  /*
  if (nmarker > nobs)
  {
    for (int i = nobs; i < nmarker; i++)
    {
      marker.ns = "basic_shapse";
      marker.id = i;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::DELETE;

      publisher_marker_.publish(marker);
    }
  }
  */
  
  return UFV::OK;
}
