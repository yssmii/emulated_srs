// -*- C++ -*-
/**
 * @file  obstacle_detector_unregistered.cpp
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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/core.hpp>

#include "obstacle_detector_unregistered.h"

#include <emulated_srs/ClassifiedObstacle.h>
#include <emulated_srs/ClassifiedObstacleArray.h>

const float emulated_srs::ObstacleDetectorUnregistered::TC_OPT_THRESHOLD_ZKEY = 1500.0;
const float emulated_srs::ObstacleDetectorUnregistered::TC_OPT_THRESHOLD_GAP = 100.0;
const float emulated_srs::ObstacleDetectorUnregistered::TC_OPT_MIN_OVERLAP_RATE = 0.8;
const std::string emulated_srs::ObstacleDetectorUnregistered::TC_OPT_MASKFILE = "MASK.png";

/*!
 *  @brief main function
*/
int main(int argc, char** argv)
{
  ros::init (argc, argv, "obstacle_detector_unregistered");

  emulated_srs::ObstacleDetectorUnregistered obstacle_detector;

  ros::spin();

  return 0;
}

emulated_srs::ObstacleDetectorUnregistered::ObstacleDetectorUnregistered(void)
    :
    param_zkey_(TC_OPT_THRESHOLD_ZKEY),
    param_gap_(TC_OPT_THRESHOLD_GAP),
    param_min_size_(TC_OPT_MIN_BLOBSIZE),
    param_min_overlap_(TC_OPT_MIN_OVERLAP_RATE),
    param_use_mask_p_(0),
    param_display_images_p_(1),
    param_publish_images_p_(0),
    param_publish_markers_p_(0),
    param_experimental_doublecheck_p_(false),
    flg_initialized_p_(false),
    node_handle_("~"),
    image_transport_(node_handle_)
{
  int param_doublecheck=0;
  
  //.launchからの読み出し
  node_handle_.getParam("zkey", param_zkey_);
  node_handle_.getParam("min_gap_of_occluding_boundary", param_gap_);
  node_handle_.getParam("min_pixels_as_object", param_min_size_);
  node_handle_.getParam("min_overlap_rate", param_min_overlap_);
  node_handle_.getParam("use_mask_p", param_use_mask_p_);
  node_handle_.getParam("display_images_p", param_display_images_p_);
  node_handle_.getParam("publish_images_p", param_publish_images_p_);
  node_handle_.getParam("publish_markers_p", param_publish_markers_p_);
  node_handle_.getParam("experimental_doublecheck_p", param_doublecheck);

  //確認のためコンソールに表示
  ROS_INFO("zkey: %f", param_zkey_);
  ROS_INFO("min_gap_of_occluding_boundary: %f", param_gap_);
  ROS_INFO("min_pixels_as_object: %d", param_min_size_);
  ROS_INFO("min_overlap_rate: %f", param_min_overlap_);
  ROS_INFO("use_mask_p: %d", param_use_mask_p_);
  ROS_INFO("display_images_p: %d", param_display_images_p_);
  ROS_INFO("publish_images_p: %d", param_publish_images_p_);
  ROS_INFO("publish_markers_p: %d", param_publish_markers_p_);
  ROS_INFO("experimental_doublecheck_p: %d", param_doublecheck);
  param_experimental_doublecheck_p_ = param_doublecheck;

  publisher_marker_ = node_handle_.advertise<visualization_msgs::Marker>(
      "/emulated_srs/visualization_marker", 1);

  publisher_obstacle_ = node_handle_.advertise<emulated_srs::ClassifiedObstacleArray>(
      "/emulated_srs/obstacles", 1);

  publisher_image_depth_ = image_transport_.advertise(
      "/emulated_srs/image_depth_classified", 1);

  ROS_INFO("publishers: OK");

  timestamp_pointcloud2_subscribed_ = ros::Time::now();

  subscriber_ = node_handle_.subscribe("/camera/depth/points", 1,
                                       &emulated_srs::ObstacleDetectorUnregistered::pc2Callback, this);
  ROS_INFO("subscriber: OK");
}

/*!
 * @if jp
 *
 * @brief pointcloud2データをサブスクライブした際に呼び出されるコールバック関数
 *
 * pointcloud2データを深度画像に変換し、さらに画像処理で領域に分割する。
 * 分割された領域の一つ一つが検知物体となる。
 * 検知物体に距離、座標情報を追加してROSメッセージとしてパブリッシュする。
 * このクラスのメインとなる処理部
 *
 * @param[in] pc2 他のROSモジュール（主にセンサ）がpublishしたpointcloud2データ
 * @return なし
 *
 * @else
 * @endif
 */
void emulated_srs::ObstacleDetectorUnregistered::pc2Callback(
    const sensor_msgs::PointCloud2ConstPtr &pc2)
{
  bool bret;
  int object_count;
  std::vector<eSRS::ObstacleClassified> obstacle_classified;  //障害物検知データ用オブジェクト

  ROS_INFO_ONCE("pointcloud received: %d %d", pc2->width, pc2->height);

  timestamp_pointcloud2_subscribed_ = ros::Time::now();

  if(pc2->height <= 1 || pc2->width <= 1)
  {
    // 高さまたは幅1のPC2(unorganized)は画像処理できないので却下
    ROS_WARN("An unorganized PC2 has subscribed: %d x %d", 
             pc2->width, pc2->height);
    return;
  }

  // Mapオブジェクトの初期化と初期設定を行う。
  // このコールバック関数で初めてサブスクライブした
  // 画像のサイズがわかるため、このタイミングで初期化が必要
  if (flg_initialized_p_ == false)
  {
    initializeMap(pc2->width, pc2->height);
    ROS_INFO_ONCE("initialize completed");
  }

  //pointcloud2からデータを抜き取り画像データ深度を作る。
  bret = convertPC2ToMapData(pc2);
  if (bret == false)
  {
    //失敗した原因については各関数内でROS_WARN(),ROS_ERROR()で出力している
    //ので、ここでは出力しない
    return;
  }
  ROS_INFO_ONCE("data conversion completed");

  //マップデータにマスク処理を行う
  setMaskToMapData();
  ROS_INFO_ONCE("masking completed");

  //範囲外の値の色付け、マスク外範囲の色付けなどを行った表示用画像データを
  //作成する
  map_for_detection_.normalize(map_for_showing_depth_data_);
  ROS_INFO_ONCE("showing data prepared");

  //物体検出を実施
  object_count = execObstacleDetection();
  //map_for_detection_.display("Det", -1);

  ROS_INFO_ONCE("obstacle detection completed: %d", object_count);

  //検知・分類された物体のリストを取得
  map_for_detection_.getObstacleClassified(obstacle_classified);

  ROS_INFO_ONCE("list prepared");

  timestamp_detection_result_published_ = ros::Time::now();

  // 表示用データの作成と表示。publishAllより先にcallすること
  displayAll();

  // 各種データのパブリッシュ
  publishAll(obstacle_classified, object_count);

  ROS_INFO_ONCE("published");

  return;
}

/*!
 * @if jp
 *
 * @brief 深度画像用のオブジェクトのインスタンス化と初期設定を行う
 * @param[in] width 画像の幅
 * @param[in] height 画像の高さ
 * @note ROSの機能により、.launchファイルでparam_xxの値は変更できます。
 *
 * @endif
 */
void emulated_srs::ObstacleDetectorUnregistered::initializeMap(
  const int width,
  const int height)
{
  map_for_detection_.reshape(width, height);
  ROS_INFO_ONCE("ClassificationMap");

  map_for_showing_depth_data_.reshape(width, height, IMAGE_N_CHANNELS);  
  ROS_INFO_ONCE("reshape");

  map_for_detection_.setZkey(param_zkey_);                  // mm
  map_for_detection_.setMinGap(param_gap_);                 // mm
  map_for_detection_.setMinSize(param_min_size_);           // pixel
  map_for_detection_.setMinOverlapRate(param_min_overlap_); // 0..1
  if (param_use_mask_p_ != 0)
  {
    map_for_detection_.setMaskImage(TC_OPT_MASKFILE);
  }

  flg_initialized_p_ = true;

  return;
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
bool emulated_srs::ObstacleDetectorUnregistered::convertPC2ToMapData(
    const sensor_msgs::PointCloud2ConstPtr &point_cloud2)
{
  unsigned long x_offset;
  unsigned long y_offset;
  unsigned long z_offset;
  bool result;

  //pointcloud2メッセージからx,y,zの情報が保存されている配列のオフセット情報を取り出す
  result = retrievePC2OffsetInfomation(point_cloud2,
                                       x_offset, y_offset, z_offset);
  if(result == false)
  {
    return result;
  }

  //オフセット情報をもとに点群データから深度情報の2次元マップを作る （map_for_detection_に値が入る）
  result = createDepthMap(point_cloud2,
                          x_offset, y_offset, z_offset);
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
void emulated_srs::ObstacleDetectorUnregistered::setMaskToMapData()
{
  bool ret(false);

  ret = map_for_detection_.hasMaskImage();
  if (ret == true)
  {
    map_for_detection_.mask();
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
 * @retval false 解析できなかった（想定していたフォーマットでは無い）
 * @retval true 解析完了
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
 * is_bigendian: False  #エンディアン
 * point_step: 16       #1つの情報のバイトサイズ
 * row_step: 10240      #1行あたりの情報のバイトサイズ -- row_step x height = 全情報
 *                                                     -- point_step x width = row_step
 * </PRE>
 * |x   |y   |z   |other info  |  16バイトというフォーマットになっている
 * @endif
 */
bool emulated_srs::ObstacleDetectorUnregistered::retrievePC2OffsetInfomation(
    const sensor_msgs::PointCloud2ConstPtr &pc2,
    unsigned long &x_offset,
    unsigned long &y_offset,
    unsigned long &z_offset)
{
  int fsize;
  std::string str;

  fsize = pc2->fields.size();

  x_offset = 0xFFFFFFFF;
  y_offset = 0xFFFFFFFF;
  z_offset = 0xFFFFFFFF;

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
 *
 * @endif
 */
bool emulated_srs::ObstacleDetectorUnregistered::createDepthMap(
    const sensor_msgs::PointCloud2ConstPtr &pc2,
    unsigned long x_offset,
    unsigned long y_offset,
    unsigned long z_offset)
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

  //|x1|y1|z1|nx2|y2|z2|n|x3|y3|z3|・・・
  //↑処理前 ↓処理後
  //|x1|x2|x3|・・・
  //|y1|y2|y3|・・・
  //|z1|z2|z3|・・・
  map_idx = 0;
  for (int i = 0; i < pc2_data_size; i += point_step)
  {
    //メッセージのuint8配列データをfloatに変換してコピー
    x_dep[map_idx] = *((float *)(&pc2data[i + x_offset]));
    y_dep[map_idx] = *((float *)(&pc2data[i + y_offset]));
    z_dep[map_idx] = *((float *)(&pc2data[i + z_offset]));

    //x,y,zの値で評価するのはzだけで良い
    if (z_dep[map_idx] == std::numeric_limits<float>::quiet_NaN())
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
int emulated_srs::ObstacleDetectorUnregistered::execObstacleDetection(void)
{
  int object_count = map_for_detection_.detect();

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
void emulated_srs::ObstacleDetectorUnregistered::publishAll(
  const std::vector<eSRS::ObstacleClassified> &obstacle_classified,
  const int object_count)
{
  publishObstaclesMessage(obstacle_classified, object_count);

  if (param_publish_markers_p_)
  {
    publishMarkersMessage(obstacle_classified, object_count);
  }
  return;
}

void emulated_srs::ObstacleDetectorUnregistered::displayAll(void)
{
  if(!(param_display_images_p_ || param_publish_images_p_)) return;
  
  //深度画像に枠と分類情報をオーバーライド(この時点で分類作業が終わり、オブジェクトに情報を渡していること)
  map_for_detection_.drawObstacleRegion(map_for_showing_depth_data_);

  if (param_display_images_p_)
  {
    map_for_showing_depth_data_.display("Depth", 10);
  }

  return;
  
}

/*!
 * @if jp
 * 障害物検知、分類済み画像データをpublishする
 * @endif
 */
int emulated_srs::ObstacleDetectorUnregistered::publishImagesMessage(void)
{
  sensor_msgs::ImagePtr msg;

  cv::Size dsize(map_for_showing_depth_data_.width(),
                 map_for_showing_depth_data_.height());
  cv::Mat detimg(dsize, CV_8UC3, map_for_showing_depth_data_.data());

  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detimg).toImageMsg();
  publisher_image_depth_.publish(msg);

  return UFV::OK;
}

/*!
 * @if jp
 * @brief 障害物検知結果を publish する。3D座標は、右手系、センサ原点、上がz、光軸がx
 * @param[in] obstacle_classified 検知した障害物の情報
 * @param[in] object_count 検知した障害物の数
 * @note
 * rostopic echo /emulated_srs/obstacles のサンプル
 * <PRE>
 * ---
 * obstacles: 
 *   - 
 *     n: 0                              // 通し番号
 *     stamp:                            // データpublish時のタイムスタンプ
 *       secs: 1569300407
 *       nsecs: 309263605
 *     type: "person"                    // Yoloによる認識結果
 *     point:                            // 外接直方体の左上手前の点の座標
 *       x: 0.51700001955
 *       y: 0.0526745580137
 *       z: 0.309713751078
 *     scale:                            // 外接直方体の寸法
 *       x: 0.427000075579
 *       y: 0.487814188004
 *       z: 0.640797376633
 *    confidence: 0.999856948853         // Yoloによる認識の信頼性
 *   - 
 *     n: 1
 *     stamp: 
 *       secs: 1569300407
 *       nsecs: 309263605
 *     type: "unknown"
 *     point: 
 *       x: 1.44900012016
 *       y: 0.799512386322
 *       z: 0.294261574745
 *     scale: 
 *       x: 0.0489998795092
 *       y: 0.0868123173714
 *       z: 0.0640079528093
 *     confidence: 0.0
 * </PRE>
 * @endif
 */
int emulated_srs::ObstacleDetectorUnregistered::publishObstaclesMessage(
  const std::vector<eSRS::ObstacleClassified> &obstacle_classified,
  const int object_count)
{
  emulated_srs::ClassifiedObstacleArray obsmsgary;

  if (object_count <= 0)
  {
    emulated_srs::ClassifiedObstacle obsmsg;

    //obsmsg.stamp = timestamp_subscribe_pointcloud2_; // データsubscribe時刻
    obsmsg.stamp = timestamp_detection_result_published_;

    obsmsg.n = 0;
    obsmsg.type = "none";
    obsmsg.confidence = 0.0;

    obsmsg.point.x = 0.0;
    obsmsg.point.y = 0.0;
    obsmsg.point.z = 0.0;

    obsmsg.scale.x = 0.0;
    obsmsg.scale.y = 0.0;
    obsmsg.scale.z = 0.0;

    obsmsgary.obstacles.push_back(obsmsg);
  }
  else
  {
    for (int i = 0; i < object_count; i++)
    {
      emulated_srs::ClassifiedObstacle obsmsg;

      //obsmsg.stamp = timestamp_subscrib_pointcloud2_;
      obsmsg.stamp = timestamp_detection_result_published_;

      obsmsg.n = obstacle_classified[i].n;
      obsmsg.type = obstacle_classified[i].classstr;
      obsmsg.confidence = obstacle_classified[i].conf;

      obsmsg.point.x = obstacle_classified[i].bvol.z / 1000.0;
      obsmsg.point.y = -obstacle_classified[i].bvol.x / 1000.0;
      obsmsg.point.z = -obstacle_classified[i].bvol.y / 1000.0;

      obsmsg.scale.x = obstacle_classified[i].bvol.depth / 1000.0;
      obsmsg.scale.y = obstacle_classified[i].bvol.width / 1000.0;
      obsmsg.scale.z = obstacle_classified[i].bvol.height / 1000.0;

      obsmsgary.obstacles.push_back(obsmsg);
    }
  }

  publisher_obstacle_.publish(obsmsgary);

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
int emulated_srs::ObstacleDetectorUnregistered::publishMarkersMessage(
    const std::vector<eSRS::ObstacleClassified> &obs,
    const int nobs)
{
  int nmarker = marker_.size();
  marker_.clear();

  for (int i = 0; i < nobs; i++)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/camera_link";
    marker.ns = "basic_shapse";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    //marker.header.stamp = timestamp_subscrib_pointcloud2_;
    marker.header.stamp = timestamp_detection_result_published_;

    marker.pose.position.x = (obs[i].bvol.z + obs[i].bvol.depth / 2) / 1000.0;
    marker.pose.position.y = -(obs[i].bvol.x + obs[i].bvol.width / 2) / 1000.0;
    marker.pose.position.z = -(obs[i].bvol.y + obs[i].bvol.height / 2) / 1000.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = obs[i].bvol.depth / 1000.0;
    marker.scale.y = obs[i].bvol.width / 1000.0;
    marker.scale.z = obs[i].bvol.height / 1000.0;

    UFV::Color ucolor = obs[i].color;
    marker.color.r = ucolor.r / 255.0;
    marker.color.g = ucolor.g / 255.0;
    marker.color.b = ucolor.b / 255.0;
    marker.color.a = 1.0;

    publisher_marker_.publish(marker);

    marker_.push_back(marker);
  }

  if (nmarker > nobs)
  {
    for (int i = nobs; i < nmarker; i++)
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = "/camera_link";
      marker.ns = "basic_shapse";
      marker.id = i;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::DELETE;

      publisher_marker_.publish(marker);
    }
  }

  return UFV::OK;
}
