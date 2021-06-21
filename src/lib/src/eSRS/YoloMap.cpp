// -*- C++ -*-
/*!
 * @file  YoloMap.cpp
 * @brief implementation of IitensityMap extention for YOLOv3
 *
 * @author Yasushi SUMI (AIST)
 *
 * Copyright (C) 2018  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 * */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string>

#include "UFV/ImageData.h"
#include "UFV/CameraParam.h"

#include "eSRS/YoloMap.h"

#include <opencv2/opencv.hpp> 
#include <opencv2/highgui/highgui.hpp> 

static const std::string YOLO_DIR = PATH_YOLO_DIR;
static const std::string DEFAULT_DATAFILE = YOLO_DIR + "/cfg/coco.data";
static const std::string DEFAULT_CFGFILE = YOLO_DIR + "/cfg/yolov3.cfg";
static const std::string DEFAULT_WEIGHTFILE = YOLO_DIR+ "/yolov3.weights";

static float colors[6][3] = { {1,0,1},         // magenta in {B,G,R}, 
                              {0.75,0.75,0.5}, // replacing {0,0,1}, red, which indicates "unknown"
                              {0,1,1},
                              {0,1,0},
                              {1,1,0},
                              {1,0,0} };

static void
trans_color_to_intensity(UFV::ImageData<unsigned char> &mapimg,
                         const cv::Mat map_matrix,
                         const cv::Mat param_int,
                         const cv::Mat coeff_dist)
{
  int itype = (mapimg.nchannels() == 3) ? CV_8UC3 : CV_8UC1;
    
  cv::Size size(mapimg.width(), mapimg.height());
  cv::Mat simg(size, itype, mapimg.data());
  //cv::imshow("simg", simg);

  cv::Mat mimg(size, itype);
  cv::Mat dimg(size, itype, mapimg.data());
  //cv::imshow("dimg", dimg);
  
  UFV::Rect droi(0, 0, mapimg.width(), mapimg.height()); // 表示領域

  //std::cerr << "Ac=" << param_int << std::endl;
  //std::cerr << "dC=" << coeff_dist << std::endl;

  cv::undistort(simg, mimg, param_int, coeff_dist);
  //cv::imshow("mimg", mimg);

  // cv::Mat dhalf;
  //cv::resize(mimg, dhalf, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
  //cv::imshow("Undistort", dhalf);

  //std::cerr << "map=" << map_matrix << std::endl;
  cv::warpPerspective(mimg, dimg, map_matrix, size,
                      cv::INTER_LINEAR+cv::WARP_FILL_OUTLIERS);

  //mapimg->display("Perspective", -1, droi, 0.5, 255);

  return;
}

void
eSRS::YoloMap::remap(void)
{
  //std::cerr << "remap"<< std::endl;
  UFV::ImageData<unsigned char> *mapimg = this->getImageData<unsigned char>();

  trans_color_to_intensity(*mapimg,
                           m_map_matrix,
                           m_param_int,
                           m_coeff_dist);
  return;
  
}

UFV::ImageData<unsigned char> 
eSRS::YoloMap::makeMapMask(void) const
{
  //std::cerr << "makeMapMask"<< std::endl;
  UFV::ImageData<unsigned char> mapmask(this->width(), this->height(), 1);
  mapmask.fillData(255);
  trans_color_to_intensity(mapmask,
                           m_map_matrix,
                           m_param_int,
                           m_coeff_dist);

  return(mapmask);
}

void
eSRS::YoloMap::setMapMatrix(const UFV::CameraParam &src,
                            const UFV::CameraParam &dst)
{
  double color_int[9];
  double color_ext[12];
  double color_dist[4];
  double ir_int[9];
  double ir_ext[12];
  double ir_dist[4];

  src.getDistortionCoeffs(color_dist);
  src.getIntrinsicMatrix(color_int);
  src.getExtrinsicMatrix(color_ext);
  
  dst.getDistortionCoeffs(ir_dist);
  dst.getIntrinsicMatrix(ir_int);
  dst.getExtrinsicMatrix(ir_ext);
  
  cv::Mat Ac(3, 3, CV_64FC1, color_int);
  //std::cerr << "Ac=" << Ac << std::endl;
  cv::Mat Mc(3, 4, CV_64FC1, color_ext);
  //std::cerr << "Mc=" << Mc << std::endl;
  cv::Mat Pc = Ac * Mc;
  //std::cerr << "Pc1=" << Pc << std::endl;
  Pc = Pc / Pc.at<double>(2,3);
  //std::cerr << "Pc2=" << Pc << std::endl;

  cv::Mat Ai(3, 3, CV_64FC1, ir_int);
  cv::Mat Mi(3, 4, CV_64FC1, ir_ext);
  cv::Mat Pi = Ai * Mi;
  Pi = Pi / Pi.at<double>(2,3);

  cv::Mat X[4]; // チェスボードの四隅
  X[0] = (cv::Mat_<double>(4,1) << 0, 0, 0, 1);
  X[1] = (cv::Mat_<double>(4,1) << 450, 0, 0, 1);
  X[2] = (cv::Mat_<double>(4,1) << 0, 300, 0, 1);
  X[3] = (cv::Mat_<double>(4,1) << 450, 300, 0, 1);

  cv::Point2f pc[4]; // src
  cv::Point2f pi[4]; // dst

  for(int i=0; i<4; i++)
  {
    cv::Mat xc = Pc*X[i];
    xc = xc / xc.at<double>(2,0);
    pc[i] = cv::Point2f(xc.at<double>(0,0), xc.at<double>(1,0));
    //std::cerr << "pc=" << pc[i] << std::endl;

    cv::Mat xi = Pi*X[i];
    xi = xi / xi.at<double>(2,0);
    pi[i] = cv::Point2f(xi.at<double>(0,0), xi.at<double>(1,0));
    //std::cerr << "pi=" << pi[i] << std::endl;
  }

  cv::Mat map = cv::getPerspectiveTransform(pc, pi);
  m_map_matrix = map.clone();
  //std::cerr << "map=" << m_map_matrix << std::endl;

  cv::Mat Dc(4, 1, CV_64FC1, color_dist);
  m_param_int = Ac.clone();
  m_coeff_dist = Dc.clone();

  //std::cerr << "Ac1=" << m_param_int << std::endl;
  //std::cerr << "dC1=" << m_coeff_dist << std::endl;

  return;
}

static image
get_yoloimage(UFV::ImageData<unsigned char> *src,
              int yolowidth, int yoloheight)
{
  unsigned char *data = (unsigned char *)src->data();
  int h = src->height();
  int w = src->width();
  int c = src->nchannels();
  //printf("%d %d %d\n", h, w, c);
  //int step = src->widthStep;
  int step = src->width() * c;
  int i, j, k;

  image out = make_image(w, h, c);

  for(i = 0; i < h; ++i)
  {
    for(k= 0; k < c; ++k)
    {
      for(j = 0; j < w; ++j)
      {
        out.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
      }
    }
  }

  //save_image(out, "out");
  
  rgbgr_image(out);
  //image resized = resize_image(out, yolowidth, yoloheight);
  image resized = letterbox_image(out, yolowidth, yoloheight);
  free_image(out);

  return resized;
}
  

static float
get_color(int c, int x, int max)
{
  float ratio = ((float)x/max)*5;
  int i = floor(ratio);
  int j = ceil(ratio);
  ratio -= i;
  float r = (1-ratio) * colors[i][c] + ratio*colors[j][c];
  //printf("%f\n", r);
  return r;
}

// 検出結果描画
int
eSRS::YoloMap::drawBoundingBox(UFV::ImageData<unsigned char> &bbox) const
{
  if(bbox.width() != this->width() || bbox.height() != this->height() ||
     bbox.nchannels() != 3)
  {
    return UFV::NG;
  }

  //! 検出結果描画用の Matインスタンスを生成する
  cv::Mat img(cv::Size(this->width(),this->height()), CV_8UC3, bbox.data());

  //! 検出結果を表す矩形の枠線の太さを設定する
  int lwidth = bbox.height() * 0.006;

  for(int i=0; i<m_nboxes; i++)
  {
    char labelstr[4096] = {0};
    int cls = -1;
    for(int j = 0; j < m_nclasses; ++j)
    {
      if (m_dets[i].prob[j] > m_thresh)
      {
        if (cls < 0)
        {
          strcat(labelstr, m_names[j]);
          cls = j;
        }
        else
        {
          strcat(labelstr, ", ");
          strcat(labelstr, m_names[j]);
        }
      }
    }
    
    if(cls >= 0)
    {
      int offset = cls*123457 % m_nclasses;
      float red = get_color(2, offset, m_nclasses) * 255;
      float green = get_color(1, offset, m_nclasses) * 255;
      float blue = get_color(0, offset, m_nclasses) * 255;
      cv::Scalar rgb(blue, green, red);
      
      box b = m_dets[i].bbox;

      cv::Point p1, p2;
      
      p1.x  = (b.x-b.w/2.)*bbox.width();
      p1.y = (b.y-b.h/2.)*bbox.height();
      p2.x = (b.x+b.w/2.)*bbox.width();
      p2.y  = (b.y+b.h/2.)*bbox.height();
      if(p1.x < 0) p1.x = 0;
      if(p1.y < 0) p1.y = 0;
      if(p2.x > bbox.width()-1) p2.x = bbox.width()-1;
      if(p2.y > bbox.height()-1) p2.y = bbox.height()-1;

      cv::rectangle(img, p1, p2, rgb, lwidth, 8);

      int baseline = 0;
      //double fscale = 0.8;
      double fscale = 0.5;
      int face = cv::FONT_HERSHEY_SIMPLEX;
      int thickness = lwidth/3;
      cv::Size tsize = cv::getTextSize(labelstr,
                                       face, fscale, thickness,
                                       &baseline);
      //printf("tsize: %d, %d, %d\n", tsize.width, tsize.height, baseline);

      cv::Point rectp1, rectp2;
      cv::Point textp;
      int labelheight = tsize.height+baseline+lwidth/2;
      if(labelheight <= p1.y)
      {
        rectp1 = p1 + cv::Point(-lwidth/2, lwidth/2);
        rectp2 = p1 + cv::Point(tsize.width+lwidth, -labelheight);
        textp = p1 + cv::Point(0, -baseline);
      }
      else
      {
        rectp1 = p1 + cv::Point(-lwidth/2, labelheight);
        rectp2 = p1 + cv::Point(tsize.width+lwidth, 0);
        textp = p1 + cv::Point(0, labelheight - baseline);
      }

      cv::rectangle(img, rectp1, rectp2,  rgb, cv::FILLED, 8);
      cv::putText(img, labelstr, textp, face, fscale, cv::Scalar(0,0,0),
                  thickness, cv::LINE_AA);
      //image label = get_label(m_alphabets, labelstr, (bbox.height()*.03));
      //draw_label(&img, p1.y + lwidth, p1.x, &label, rgb);
    }
  }

  return UFV::OK;
}

// 検出結果の格納
int
eSRS::YoloMap::setLabeledObjects(void)
{
  if(m_thresh <= 0.5) return 0;

  int ocount=0;

  for(int i=0; i<m_nboxes; i++)
  {
    int cls = -1;
    float tmp_thresh = m_thresh;
    for(int j = 0; j < m_nclasses; ++j)
    {
      if(m_dets[i].prob[j] > tmp_thresh)
      {
        //assert(cls < 0); // m_thresh > 0.5 であれば、複数候補はありえないはず
        cls = j;
        tmp_thresh = m_dets[i].prob[j];
        //fprintf(stdout, "* %s: %.0f%%\n", labelstr, m_dets[i].prob[j]*100);
      }
    }
    
    if(cls >= 0)
    {
      //fprintf(stdout, "* %s: %.0f%%, %d\n",
      //        m_names[cls], m_dets[i].prob[cls]*100, i);
    
      eSRS::YoloObject yobj;

      yobj.n = ocount;
      yobj.conf = m_dets[i].prob[cls];
      yobj.labelstr = m_names[cls];
      
      int offset = cls*123457 % m_nclasses;
      yobj.color.r = get_color(2, offset, m_nclasses) * 255;
      yobj.color.g = get_color(1, offset, m_nclasses) * 255;
      yobj.color.b = get_color(0, offset, m_nclasses) * 255;

      box b = m_dets[i].bbox;
      cv::Point p1, p2;
      p1.x  = (b.x-b.w/2.)*this->width();
      p1.y = (b.y-b.h/2.)*this->height();
      p2.x = (b.x+b.w/2.)*this->width();
      p2.y  = (b.y+b.h/2.)*this->height();
      if(p1.x < 0) p1.x = 0;
      if(p1.y < 0) p1.y = 0;
      if(p2.x > this->width()-1) p2.x = this->width()-1;
      if(p2.y > this->height()-1) p2.y = this->height()-1;

      yobj.bbox.x = p1.x;
      yobj.bbox.y = p1.y;
      yobj.bbox.width = p2.x - p1.x + 1;
      yobj.bbox.height = p2.y - p1.y + 1;

      m_obj_vec.push_back(yobj);

      ocount++;
    }
  }

  return ocount;
}

void
eSRS::YoloMap::detect(void)
{
  m_obj_vec.clear();
  
  if(m_dets != 0)
  {
    free_detections(m_dets, m_nboxes);
    m_dets = 0;
  }
  m_nboxes = 0;
  
  int w = this->width();
  int h = this->height();
  int c = 3;
  
  float nms = .45;
  float hier_thresh = .75;
  
  image sized = get_yoloimage(this->getImageData<unsigned char>(),
                              m_net->w, m_net->h);

  //save_image(sized, "sized");
  //exit(0);

  float *X = sized.data;
#ifdef _WIN32
  network_predict(*m_net, X);
  m_dets = get_network_boxes(m_net, w, h, m_thresh, hier_thresh, 0, 1,
                             &m_nboxes, 1);
#else
  network_predict(m_net, X);
  m_dets = get_network_boxes(m_net, w, h, m_thresh, hier_thresh, 0, 1,
                             &m_nboxes);
#endif

  //printf("Object: %d\n", m_nboxes);
  
  if (nms) do_nms_sort(m_dets, m_nboxes, m_nclasses, nms);
  //draw_detections(im, dets, nboxes, thresh, names, alphabet, l.classes);

  free_image(sized);

  this->setLabeledObjects();

  return;
}

void
eSRS::YoloMap::load_files(void)
{
  //cuda_set_device(0);
  
  list *options = read_data_cfg(const_cast<char*>(DEFAULT_DATAFILE.c_str()));
  char *name_list = option_find_str(options, "names", "data/names.list");
  m_names = get_labels(name_list);

  m_net = load_network(const_cast<char*>(DEFAULT_CFGFILE.c_str()),
                       const_cast<char*>(DEFAULT_WEIGHTFILE.c_str()), 0);
  set_batch_network(m_net, 1);
  srand(2222222);

  if(options) free(options);

  return;
}

