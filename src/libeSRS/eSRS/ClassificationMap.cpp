// -*- C++ -*-
/*!
 * @file  ClassificationMap.cpp
 * @brief ClassificationMap class implementation
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

#include "UFV/ImageData.h"
#include "UFV/CameraParam.h"

#include "eSRS/Object.h"
#include "eSRS/ClassificationMap.h"

#include <opencv2/opencv.hpp> 

// 80%以上重なってたら同じ物体とみなす
const double eSRS::ClassificationMap::DEFAULT_MIN_OVERLAP_RATE=0.8;

static float
calc_overlap_rate(eSRS::BoundingBox bbox1, eSRS::BoundingBox bbox2)
{
  int sx = MAX(bbox1.x, bbox2.x);
  int sy = MAX(bbox1.y, bbox2.y);
  int ex = MIN(bbox1.x + bbox1.width, bbox2.x + bbox2.width);
  int ey = MIN(bbox1.y + bbox1.height, bbox2.y + bbox2.height);

  int w = ex - sx;
  int h = ey - sy;

  if(w > 0 && h > 0)
  {
    int bboxsize = bbox1.width * bbox1.height + bbox2.width * bbox2.height;
    if(bboxsize > 0)
      return(w*h*2.0 / bboxsize);
  }
  
  return(0.0);
}
  
int
eSRS::ClassificationMap::detect(std::vector<YoloObject> &yobjs,
                                const bool flg_doublecheck)
{
  if(flg_doublecheck == false || yobjs.size() == 0)
    return this->detect(yobjs);
  
  std::vector<ObstacleClassified> ocls_vec_ext;
  ocls_vec_ext.clear();

  this->setZkey(this->getZkey()+1000.0); // 物体の枠を決めるための予備検出
  int nobs_ext = eSRS::DetectionMap::detect();
  
  if(nobs_ext > 0)
  {
    //std::vector<eSRS::ObstacleClassified> oclassvec_ext;
    for(int j=0; j<nobs_ext; j++)
    {
      eSRS::ObstacleClassified oclass_ext(m_obs_vec[j]);

      oclass_ext.n = j;
      for(uint i=0; i<yobjs.size(); i++)
      {
        eSRS::YoloObject yobj = yobjs[i];

        float overlap = calc_overlap_rate(oclass_ext.bbox, yobj.bbox);
        if(overlap > this->m_min_overlap_rate && oclass_ext.overlap < overlap)
        {
          oclass_ext.overlap = overlap;
          oclass_ext.conf = yobj.conf;
          oclass_ext.classstr = yobj.labelstr;
          oclass_ext.color = yobj.color;
        }
      }

      ocls_vec_ext.push_back(oclass_ext);
    }
  }

  m_obs_vec.clear(); // 一旦クリア
  this->setZkey(this->getZkey()-1000.0);

  m_ocls_vec.clear();
  
  int nobs = eSRS::DetectionMap::detect(); // 正位置で再度検出

  //UFV::Rect droi(120, 60, this->width()-320, this->height()-120); 
  //m_labeled.display("Labeled", -1,  droi, DISPLAY_MAGNIFICATION, 16);

  //std::cout << "Yolo Objects: " << yobjs.size() << std::endl;
  //this->printObstacles(stdout);

  if(nobs <= 0) return 0;

  //std::vector<eSRS::ObstacleClassified> oclassvec;
  for(int j=0; j<nobs; j++)
  {
    //eSRS::Obstacle obs = m_obs_vec[j];
    eSRS::ObstacleClassified oclass(m_obs_vec[j]);

    oclass.n = j;
    for(uint i=0; i<yobjs.size(); i++)
    {
      eSRS::YoloObject yobj = yobjs[i];

      float overlap = calc_overlap_rate(oclass.bbox, yobj.bbox);
      if(overlap > this->m_min_overlap_rate && oclass.overlap < overlap)
      {
        // TODO: oclass.conf = yobj.conf * overlap; ではどうか？
        oclass.overlap = overlap;
        oclass.conf = yobj.conf;
        oclass.classstr = yobj.labelstr;
        oclass.color = yobj.color;
      }
    }

    this->m_ocls_vec.push_back(oclass);
  }

  /*
  fprintf(stdout, "%d %d\n", nobs, nobs_ext);

  for(int j=0; j<nobs; j++)
  {
    std::cout << "O: " <<
      j << " " <<
      m_obs_vec[j].bbox.x << " " <<
      m_obs_vec[j].bbox.x + m_obs_vec[j].bbox.width << " " <<
      m_obs_vec[j].bbox.y << " " <<
      m_obs_vec[j].bbox.y + m_obs_vec[j].bbox.height << std::endl;
  }
      
  for(int j=0; j<nobs; j++)
  {
    std::cout << "A: " <<
      j << " " <<
      m_ocls_vec[j].classstr.c_str() << " " <<
      m_ocls_vec[j].bbox.x << " " <<
      m_ocls_vec[j].bbox.x + m_ocls_vec[j].bbox.width << " " <<
      m_ocls_vec[j].bbox.y << " " <<
      m_ocls_vec[j].bbox.y + m_ocls_vec[j].bbox.height << std::endl;
  }
      
  for(int j=0; j<nobs_ext; j++)
  {
    std::cout << "B: " <<
      j << " " <<
      ocls_vec_ext[j].classstr.c_str() << " " <<
      ocls_vec_ext[j].bbox.x << " " <<
      ocls_vec_ext[j].bbox.x + ocls_vec_ext[j].bbox.width << " " <<
      ocls_vec_ext[j].bbox.y << " " <<
      ocls_vec_ext[j].bbox.y + ocls_vec_ext[j].bbox.height << std::endl;
  }
  */
      
  for(int j=0; j<nobs; j++)
  {
    if(m_ocls_vec[j].classstr == "unknown")
    {
      for(int i=0; i<nobs_ext; i++)
      {
        if(ocls_vec_ext[i].classstr != "unknown")
        {
          if(ocls_vec_ext[i].bbox.x <= m_ocls_vec[j].bbox.x + 0.01 &&
             ocls_vec_ext[i].bbox.y <= m_ocls_vec[j].bbox.y + 0.01 &&
             ocls_vec_ext[i].bbox.x + ocls_vec_ext[i].bbox.width + 0.01 >=
             m_ocls_vec[j].bbox.x + m_ocls_vec[j].bbox.width &&
             ocls_vec_ext[i].bbox.y + ocls_vec_ext[i].bbox.height + 0.01 >=
             m_ocls_vec[j].bbox.y + m_ocls_vec[j].bbox.height)
          {
            m_ocls_vec[j].classstr = ocls_vec_ext[i].classstr;
            m_ocls_vec[j].color = ocls_vec_ext[i].color;
            break;
          }
        }
      }
    }
    
  }
  

  return nobs;
}

int
eSRS::ClassificationMap::detect(std::vector<YoloObject> &yobjs)
{
  m_ocls_vec.clear();
  
  int nobs = eSRS::DetectionMap::detect();

  //UFV::Rect droi(120, 60, this->width()-320, this->height()-120); 
  //m_labeled.display("Labeled", -1,  droi, DISPLAY_MAGNIFICATION, 16);

  //std::cout << "Yolo Objects: " << yobjs.size() << std::endl;
  //this->printObstacles(stdout);

  if(nobs <= 0) return 0;

  //std::vector<eSRS::ObstacleClassified> oclassvec;
  for(int j=0; j<nobs; j++)
  {
    //eSRS::Obstacle obs = m_obs_vec[j];
    eSRS::ObstacleClassified oclass(m_obs_vec[j]);

    oclass.n = j;
    for(uint i=0; i<yobjs.size(); i++)
    {
      eSRS::YoloObject yobj = yobjs[i];

      float overlap = calc_overlap_rate(oclass.bbox, yobj.bbox);
      if(overlap > this->m_min_overlap_rate && oclass.overlap < overlap)
      {
        // TODO: oclass.conf = yobj.conf * overlap; ではどうか？
        oclass.overlap = overlap;
        oclass.conf = yobj.conf;
        oclass.classstr = yobj.labelstr;
        oclass.color = yobj.color;
      }
    }

    this->m_ocls_vec.push_back(oclass);
  }

  return nobs;
}

int
eSRS::ClassificationMap::detect(void)
{
  this->m_ocls_vec.clear();
  
  int nobs = eSRS::DetectionMap::detect();
  if(nobs <= 0) return 0;

  for(int j=0; j<nobs; j++)
  {
    eSRS::ObstacleClassified oclass(m_obs_vec[j]);

    oclass.n = j;

    this->m_ocls_vec.push_back(oclass);
  }

  return nobs;
}

static void
get_xyz_label(const eSRS::BoundingVolume vol,
              const double fscale,
              const int face,
              const int thickness,
              const int lwidth,
              const cv::Point p1,
              const cv::Point p2,
              const std::string oclassstr,
              std::string &labelstr,
              cv::Point &rectp1, cv::Point &rectp2,
              cv::Point &textp_cls, cv::Point &textp_xyz)
{
  //label = std::to_string((int)std::round(vol.z));
  labelstr = std::to_string((int)std::round(vol.z)) + ","
    + std::to_string((int)std::round(vol.width)) + "x"
    + std::to_string((int)std::round(vol.height));

  //std::cout << ">>> " << label << std::endl;

  int baseline = 0;
  cv::Size tsize_xyz = cv::getTextSize(labelstr,
                                       face, fscale, thickness,
                                       &baseline);
  cv::Size tsize_cls = cv::getTextSize(oclassstr,
                                       face, fscale, thickness,
                                       &baseline);
  cv::Size tsize = (tsize_xyz.width > tsize_cls.width) ? tsize_xyz : tsize_cls;
  int labelheight = tsize.height+baseline+lwidth/2;
  int labelhwidth = (tsize.width)/2;
  cv::Point c1 = cv::Point((p1.x+p2.x)/2, (p1.y+p2.y)/2);

  rectp1 = c1 + cv::Point(-labelhwidth, (labelheight*3)/2);
  rectp2 = c1 + cv::Point(labelhwidth, -labelheight/2);
  textp_cls = c1 + cv::Point(-labelhwidth, -baseline + labelheight/2);
  textp_xyz = c1 + cv::Point(-labelhwidth, -baseline + (labelheight*3)/2);
  
  return;
}


static void
get_reg_label(const eSRS::BoundingVolume vol,
              const double fscale1, // classname
              const double fscale2, // xyz
              const int face,
              const int thickness,
              const int lwidth,
              const cv::Point c1,
              const std::string oclassstr,
              std::string &labelstr,
              cv::Point &rectp1, cv::Point &rectp2,
              cv::Point &textp_cls, cv::Point &textp_xyz)
{
  //label = std::to_string((int)std::round(vol.z));
  labelstr = std::to_string((int)std::round(vol.z)) + ","
    + std::to_string((int)std::round(vol.width)) + "x"
    + std::to_string((int)std::round(vol.height));

  //std::cout << ">>> " << label << std::endl;

  int baseline1 = 0;
  int baseline2 = 0;
  cv::Size tsize_xyz = cv::getTextSize(labelstr,
                                       face, fscale2, thickness,
                                       &baseline2);
  cv::Size tsize_cls = cv::getTextSize(oclassstr,
                                       face, fscale1, thickness,
                                       &baseline1);
  int labelwidth = (tsize_xyz.width > tsize_cls.width) ? tsize_xyz.width : tsize_cls.width;

  /* std::cout << ">>> ("
            << tsize_cls.height << ", "
            << baseline1 << "), ("
            << tsize_xyz.height << ", "
            << baseline2 << ")"
            << std::endl; */
  
  labelwidth += lwidth*2;
  //int labelheight1 = tsize_cls.height+baseline1+lwidth/2;
  //int labelheight2 = tsize_xyz.height+baseline2+lwidth/2;
  int labelheight1 = tsize_cls.height+lwidth+baseline1;
  int labelheight2 = tsize_xyz.height+lwidth;
  int labelhwidth = labelwidth/2;

  rectp1 = c1 + cv::Point(-labelhwidth, labelheight2);
  rectp2 = c1 + cv::Point(labelhwidth, -labelheight1);
  //textp_cls = c1 + cv::Point(-labelhwidth+lwidth, -baseline1);
  //textp_xyz = c1 + cv::Point(-labelhwidth+lwidth, -lwidth + labelheight2);
  textp_cls = c1 + cv::Point(-tsize_cls.width/2, -baseline1);
  textp_xyz = c1 + cv::Point(-tsize_xyz.width/2, -lwidth + labelheight2);

  // TODO: ラベルが画像の境界をはみ出るときの処理
  
  return;
}

int
eSRS::ClassificationMap::drawBoundingBox(UFV::ImageData<unsigned char> &bbox)
  const
{
  //! 検出結果描画先と Depth Mapの width、height、チャンネル数の
  //  いずれかが異なる場合、エラー復帰する
  if(bbox.width() != this->width() || bbox.height() != this->height() ||
     bbox.nchannels() != 3)
  {
    return UFV::NG;
  }

  //! 検出結果描画用の Matインスタンスを生成する
  cv::Mat img(cv::Size(this->width(),this->height()), CV_8UC3, bbox.data());

  //! 検出結果を表す矩形の枠線の太さを設定する
  int lwidth = bbox.height() * 0.006;
  // lwidth /= mag;

  //if(bbox.width() <= 320 || bbox.height() <= 240) lwidth = 1;

  cv::Point p1, p2;

  //! 検出結果数分、以下の処理を行う
  for(unsigned int i=0; i<m_obs_vec.size(); i++)
  {
    //! - 矩形の頂点と反対側の頂点を算出する
    p1.x = m_obs_vec[i].bbox.x;
    p1.y = m_obs_vec[i].bbox.y;
    p2.x = m_obs_vec[i].bbox.x + m_obs_vec[i].bbox.width - 1;
    p2.y = m_obs_vec[i].bbox.y + m_obs_vec[i].bbox.height - 1;

    //! - 画像に矩形を描画する
    UFV::Color ucolor = m_ocls_vec[i].color;
    cv::rectangle(img, p1, p2, cv::Scalar(ucolor.b, ucolor.g, ucolor.r),
                  lwidth, 8);

    if(this->m_flg_drawlabel)
    {
      //double fscale = 0.5/mag;
      cv::Point pgrv;
      pgrv.x = (p1.x + p2.x)/2;
      pgrv.y = (p1.y + p2.y)/2;

      double fscale1 = 0.5;
      double fscale2 = 0.35;
      int face = cv::FONT_HERSHEY_SIMPLEX;
      int thickness = lwidth/3;
      std::string xyzlabel;
      cv::Point rectp1, rectp2, textp_xyz, textp_cls;
      //get_xyz_label(m_obs_vec[i].bvol, fscale, face, thickness, lwidth,
      //              p1, p2, m_ocls_vec[i].classstr,
      //              xyzlabel, rectp1, rectp2,
      //              textp_cls, textp_xyz);
      get_reg_label(m_obs_vec[i].bvol, fscale1, fscale2,
                    face, thickness, lwidth,
                    pgrv, m_ocls_vec[i].classstr,
                    xyzlabel, rectp1, rectp2,
                    textp_cls, textp_xyz);
      
      cv::rectangle(img, rectp1, rectp2,
                    cv::Scalar(ucolor.b, ucolor.g, ucolor.r),
                    cv::FILLED, 8);
      cv::putText(img, m_ocls_vec[i].classstr.c_str(),
                  textp_cls, face, fscale1,
                  cv::Scalar(0,0,0), thickness, cv::LINE_AA);
      cv::putText(img, xyzlabel.c_str(),
                  textp_xyz, face, fscale2,
                  cv::Scalar(0,0,0), thickness, cv::LINE_AA);
    }
  }

  //this->printObstacles(stdout);

  return UFV::OK;
}

int
eSRS::ClassificationMap::drawObstacleRegion(UFV::ImageData<unsigned char> &bbox)
  const
{
  //! 検出結果描画先と Depth Mapの width、height、チャンネル数の
  //  いずれかが異なる場合、エラー復帰する
  if(bbox.width() != this->width() || bbox.height() != this->height() ||
     bbox.nchannels() != 3)
  {
    return UFV::NG;
  }

  //! 検出結果描画用の Matインスタンスを生成する
  cv::Mat img(cv::Size(this->width(),this->height()), CV_8UC3, bbox.data());

  short *plbl = m_labeled.data();
  uchar *pbox = bbox.data();
  for(int i=0; i<this->width()*this->height(); i++)
  {
    if(*plbl > 0)
    {
      if(*plbl <= m_ocls_vec.size()) // 不要のはず
      {
        UFV::Color ucolor = m_ocls_vec[*plbl-1].color;
        *pbox *= ucolor.b/255.0;
        *(pbox+1) *= ucolor.g/255.0;
        *(pbox+2) *= ucolor.r/255.0;
      }
    }
    pbox += 3;
    plbl++;
  }

  if(this->m_flg_drawlabel)
  {
    //! 検出結果を表す矩形の枠線の太さを設定する
    int lwidth = bbox.height() * 0.005;

    //! 検出結果数分、以下の処理を行う
    for(unsigned int i=0; i<m_obs_vec.size(); i++)
    {
    //! - 矩形の頂点と反対側の頂点を算出する
      cv::Point p1;
      p1.x = m_obs_vec[i].igrv.x;
      p1.y = m_obs_vec[i].igrv.y;

      //! - 画像に矩形を描画する
      UFV::Color ucolor = m_ocls_vec[i].color;

      //double fscale = 0.5/mag;
      double fscale1 = 0.5;
      double fscale2 = 0.35;
      int face = cv::FONT_HERSHEY_SIMPLEX;
      int thickness = lwidth/3;
      std::string xyzlabel;
      cv::Point rectp1, rectp2, textp_xyz, textp_cls;
      get_reg_label(m_obs_vec[i].bvol, fscale1, fscale2,
                    face, thickness, lwidth,
                    p1, m_ocls_vec[i].classstr,
                    xyzlabel, rectp1, rectp2,
                    textp_cls, textp_xyz);
      
      cv::rectangle(img, rectp1, rectp2,
                    cv::Scalar(ucolor.b, ucolor.g, ucolor.r),
                    cv::FILLED, 8);
      cv::putText(img, m_ocls_vec[i].classstr.c_str(),
                  textp_cls, face, fscale1,
                  cv::Scalar(0,0,0), thickness, cv::LINE_AA);
      cv::putText(img, xyzlabel.c_str(),
                  textp_xyz, face, fscale2,
                  cv::Scalar(0,0,0), thickness, cv::LINE_AA);
                  
    }
  }

  //this->printObstacles(stdout);

  return UFV::OK;
}
