// -*- C++ -*-
/*!
 * @file  YoloMap.h
 * @brief definition of IntensityMap extention for Yolo
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * Copyright (C) 2018  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifndef __LIBESRS_ESRS_YOLOMAP_H__
#define __LIBESRS_ESRS_YOLOMAP_H__

#include "UFV/ImageData.h"
#include "UFV/IntensityMap.h"
#include "UFV/CameraParam.h"

#include "eSRS/Object.h"
#include "eSRS/DetectionMap.h"

//#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp> 

#define GPU 1
#define OPENCV 1
#define CUDNN 1

#define BLOCK 512

#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"

#ifdef CUDNN
#include "cudnn.h"
#endif

//#include "darknet.h"
//#include "network.h"
//#include "image.h"
//#include "option_list.h"

extern "C" {  
#ifdef _WIN32
#define _TIMESPEC_DEFINED
//#include "darknet.h"
#include "network.h"
#include "image.h"
#include "option_list.h"

#else
#include "darknet.h"
#endif
}

namespace eSRS
{

class YoloMap : public UFV::IntensityMap
{
private:
  
public:
  YoloMap(void)
      : m_names(0),
        m_net(0),
        m_dets(0),
        m_nboxes(0),
        m_thresh(.75)
    {
      this->load_files();
      //m_alphabets = load_alphabet();
      m_nclasses = (m_net->layers[m_net->n-1]).classes; // m_net->n > 0 ?
      m_obj_vec.clear();
    };
  YoloMap(const int width, const int height)
      : UFV::IntensityMap(width, height, 3, TYPE_U8),
        m_names(0),
        m_net(0),
        m_dets(0),
        m_nboxes(0),
        m_thresh(.75)
    {
      this->load_files();
      //m_alphabets = load_alphabet();
      m_nclasses = (m_net->layers[m_net->n-1]).classes; // m_net->n > 0 ?
      m_obj_vec.clear();
    };
  ~YoloMap(void) {};
  
public:
  void detect(void);

public:
  int drawBoundingBox(UFV::ImageData<unsigned char> &bbox) const;
  void getLabeledObject(std::vector<YoloObject> &objs) const
    {
      objs = m_obj_vec;
    };
 
  //std::vector<YoloObject> getLabeledObject(void) const { return m_obj_vec; };

public:
  virtual void reshape(const int width, const int height) {
    this->UFV::IntensityMap::reshape(width, height, 3, TYPE_U8);
    m_obj_vec.clear();
    return;
  };

public:
  void remap(void);
  void setMapMatrix(const UFV::CameraParam &src, const UFV::CameraParam &dst);
  UFV::ImageData<unsigned char> makeMapMask(void) const;

private:
  int setLabeledObjects(void);

private:
  void load_files(void);

private:
  char **m_names; // [BUG] メモリリーク状態。要free()
  network *m_net;
  detection *m_dets;
  //image **m_alphabets;

  int m_nclasses;
  int m_nboxes;

  float m_thresh;

  cv::Mat m_map_matrix;
  cv::Mat m_coeff_dist;
  cv::Mat m_param_int;

protected:
  std::vector<YoloObject> m_obj_vec;
};

} // namespace eSRS

#endif // __LIBESRS_ESRS_YOLOMAP_H__
