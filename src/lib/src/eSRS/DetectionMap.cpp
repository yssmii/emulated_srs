/*!
 * @file  DetectionMap.cpp
 * @brief Implementation of DetectionMap class
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 *
 * @note ラベリングアルゴリズムは、画像認識論, 長尾著, コロナ社 (1983) による
 *
 * Copyright (C) 2013  AIST
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <assert.h>
#include <vector>
#include <opencv2/opencv.hpp> 
#include <opencv2/highgui/highgui.hpp> 

#include "UFV/types.h"
#include "UFV/keydefs.h"
#include "UFV/ImageData.h"

#include "eSRS/DepthMap.h"
#include "eSRS/DetectionMap.h"

#define DEBUG 1
//#define DEBUG1 1
//#define DEBUG2 1

#if HAVE_LIMITS_H
# include <limits.h>
# define TC_MINSHORT   SHRT_MIN
# define TC_MAXSHORT   SHRT_MAX
# define TC_MININT     INT_MIN
# define TC_MAXINT     INT_MAX
# define TC_MINLONG    LONG_MIN
# define TC_MAXLONG    LONG_MAX
#elif HAVE_VALUES_H
# include <values.h>
# define TC_MINSHORT   MINSHORT
# define TC_MAXSHORT   MAXSHORT
# define TC_MININT     MININT
# define TC_MAXINT     MAXINT
# define TC_MINLONG    MINLONG
# define TC_MAXLONG    MAXLONG
#else /* HAVE_VALUES_H */
# ifndef SIZEOF_LONG
#   define TC_MININT   (-2147483647-1)
#   define TC_MAXINT   2147483647
#   define TC_MINLONG  (-2147483647-1)
#   define TC_MAXLONG  2147483647
#   define TC_MINSHORT (-32768)
#   define TC_MAXSHORT 32767
# else
#  if SIZEOF_INT == 8
#   define TC_MININT   (-9223372036854775807-1)
#   define TC_MAXINT   9223372036854775807
#  elif SIZEOF_INT == 4
#   define TC_MININT   (-2147483647-1)
#   define TC_MAXINT   2147483647
#  elif SIZEOF_INT == 2
#   define TC_MININT   (-32768)
#   define TC_MAXINT   32767
#  elif SIZEOF_INT == 1
#   define TC_MININT   (-128)
#   define TC_MAXINT   127
#  endif
#  if SIZEOF_LONG == 8
#   define TC_MINLONG  (-9223372036854775807-1)
#   define TC_MAXLONG  9223372036854775807
#  elif SIZEOF_LONG == 4
#   define TC_MINLONG  (-2147483647-1)
#   define TC_MAXLONG  2147483647
#  elif SIZEOF_LONG == 2
#   define TC_MINLONG  (-32768)
#   define TC_MAXLONG  32767
#  elif SIZEOF_LONG == 1
#   define TC_MINLONG  (-128)
#   define TC_MAXLONG  127
#  endif
#  if SIZEOF_SHORT == 8
#   define TC_MINSHORT (-9223372036854775807-1)
#   define TC_MAXSHORT 9223372036854775807
#  elif SIZEOF_SHORT == 4
#   define TC_MINSHORT (-2147483647-1)
#   define TC_MAXSHORT 2147483647
#  elif SIZEOF_SHORT == 2
#   define TC_MINSHORT (-32768)
#   define TC_MAXSHORT 32767
#  elif SIZEOF_SHORT == 1
#   define TC_MINSHORT (-128)
#   define TC_MAXSHORT 127
#  endif
# endif
#endif

#if HAVE_FLOAT_H
# include <float.h>
#  define TC_MINFLOAT  (-FLT_MAX)
#  define TC_MAXFLOAT  FLT_MAX
#  define TC_MINDOUBLE (-DBL_MAX)
#  define TC_MAXDOUBLE DBL_MAX
#elif HAVE_VALUES_H
# if HAVE_LIMITS_H
#  include <values.h>
# endif /* HAVE_LIMITS_H */
#  define TC_MINFLOAT  (-MAXFLOAT)
#  define TC_MAXFLOAT  MAXFLOAT
#  define TC_MINDOUBLE (-MAXDOUBLE)
#  define TC_MAXDOUBLE MAXDOUBLE
#else /* HAVE_VALUES_H */
# ifndef SIZEOF_DOUBLE
#   define TC_MINDOUBLE (-1.7976931348623157E+308)
#   define TC_MAXDOUBLE 1.7976931348623157E+308
#   define TC_MINFLOAT  (-3.402823466E+38)
#   define TC_MAXFLOAT  3.402823466E+38
# else
#  if SIZEOF_DOUBLE == 8
#   define TC_MINDOUBLE (-1.7976931348623157E+308)
#   define TC_MAXDOUBLE 1.7976931348623157E+308
#  elif SIZEOF_DOUBLE == 4
#   define TC_MINDOUBLE (-3.402823466E+38)
#   define TC_MAXDOUBLE 3.402823466E+38
#  endif
#  if SIZEOF_FLOAT == 8
#   define TC_MINFLOAT  (-1.7976931348623157E+308)
#   define TC_MAXFLOAT  1.7976931348623157E+308
#  elif SIZEOF_FLOAT == 4
#   define TC_MINFLOAT  (-3.402823466E+38)
#   define TC_MAXFLOAT  3.402823466E+38
#  endif
# endif
#endif /* HAVE_VALUES_H */

#define _MAXIMUM_LABEL TC_MAXSHORT // ラベリング最大数 TODO:可変にすべき?

// Zキーの閾値距離 (1500.0 mm)
const double eSRS::DetectionMap::DEFAULT_ZKEY = 1500.0;

// これより離れていれば別物体(100.0 mm)
const double eSRS::DetectionMap::DEFAULT_MIN_GAP = 100.0;

namespace eSRS
{
/*! @struct Blob
 *  @brief Blob
 */
struct Blob
{
  //! flg
  int flg;

  double c;
  double r;

  //! colmin
  int colmin;
  
  //! colmax
  int colmax;

  //! rowmin
  int rowmin;

  //! rowmax
  int rowmax;

  //! x
  double x;

  //! y
  double y;

  //! z
  double z;

  //! xmax
  double xmax;

  //! ymax
  double ymax;

  //! zmax
  double zmax;

  //! xmin
  double xmin;

  //! ymin
  double ymin;

  //! zmin
  double zmin;

  //! size (pixels)
  int size;
};
} // namespace eSRS

#ifdef DEBUG2

/*!
 *  @brief  検出した物体情報のファイル書き込み
 *  @param  [in] fp  出力先ファイルストリーム
 *  @param  [in] obs  検出した物体情報の配列
 *  @note
 *    - 検出した物体情報をファイルに書き込む
 *  @attention
 *    - "DEBUG2"が定義されている場合のみ定義される
 */
static void
print_boundingbox(FILE *fp,  std::vector<eSRS::Obstacle> &obs)
{
  unsigned int i;
  double xmin, ymin, zmin;
  double xmax, ymax, zmax;

  fprintf(fp, "---\n");
  for(i=0; i<obs.size(); i++)
  {
    xmin = obs[i].bvol.x;
    ymin = obs[i].bvol.y;
    zmin = obs[i].bvol.z;
    xmax = xmin + obs[i].bvol.width;
    ymax = ymin + obs[i].bvol.height;
    zmax = zmin + obs[i].bvol.depth;

    fprintf(fp, "- TYPE: line\n");
    fprintf(fp, "  DATA:\n");
    fprintf(fp, "    - [%f, %f, %f] # S\n", xmin, ymin, zmin);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmax, ymin, zmin);

    fprintf(fp, "    - [%f, %f, %f] # S\n", xmin, ymax, zmin);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmax, ymax, zmin);

    fprintf(fp, "    - [%f, %f, %f] # S\n", xmin, ymin, zmax);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmax, ymin, zmax);

    fprintf(fp, "    - [%f, %f, %f] # S\n", xmin, ymax, zmax);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmax, ymax, zmax);

    fprintf(fp, "    - [%f, %f, %f] # S\n", xmin, ymin, zmin);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmin, ymax, zmin);

    fprintf(fp, "    - [%f, %f, %f] # S\n", xmax, ymin, zmin);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmax, ymax, zmin);

    fprintf(fp, "    - [%f, %f, %f] # S\n", xmin, ymin, zmax);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmin, ymax, zmax);

    fprintf(fp, "    - [%f, %f, %f] # S\n", xmax, ymin, zmax);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmax, ymax, zmax);

    fprintf(fp, "    - [%f, %f, %f] # S\n", xmin, ymin, zmin);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmin, ymin, zmax);

    fprintf(fp, "    - [%f, %f, %f] # S\n", xmax, ymin, zmin);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmax, ymin, zmax);

    fprintf(fp, "    - [%f, %f, %f] # S\n", xmin, ymax, zmin);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmin, ymax, zmax);

    fprintf(fp, "    - [%f, %f, %f] # S\n", xmax, ymax, zmin);
    fprintf(fp, "    - [%f, %f, %f] # E\n", xmax, ymax, zmax);
  }

  return;
}

/*!
 *  @brief  検出した物体情報のファイル書き込み
 *  @param  [in] fp  出力先ファイルストリーム
 *  @param  [in] obs  検出した物体情報の配列
 *  @note
 *    - 検出した物体情報をファイルに書き込む
 *  @attention
 *    - "DEBUG2"が定義されている場合のみ定義される
 */
static void
print_obstacles(FILE *fp, std::vector<eSRS::Obstacle> &obs)
{
  fprintf(fp, "---\n");
  for(unsigned int i=0; i<obs.size(); i++)
  {
    fprintf(fp, "- ORG: [%f, %f, %f]\n", 
            obs[i].bvol.x, obs[i].bvol.y, obs[i].bvol.z);

    fprintf(fp, "  DIM: [%f, %f, %f]\n", 
            obs[i].bvol.width, obs[i].bvol.height, obs[i].bvol.depth);

    fprintf(fp, "  GRV: [%f, %f, %f]\n", 
            obs[i].grv.x, obs[i].grv.y, obs[i].grv.z);

    fprintf(fp, "  IGRV: [%f, %f]\n", 
            obs[i].igrv.x, obs[i].igrv.y);
  }

  return;
}

#endif // DEBUG2

/*!
 *  @brief  gap_p
 *  @param  [in] z_far  z_far
 *  @param  [in] z_near  z_near
 *  @param  [in] mingap  ギャップの最小値
 *  @note       N/A
 *  @attention  N/A
 */
static bool
gap_p(const float z_far, const float z_near, const double mingap)
{

  //! データなしの場合、falseを返す
  if(z_far < -FLT_EPSILON || z_near < -FLT_EPSILON)
    return false;

  //! z_near が z_far より一定距離(mingap)以上近ければtrueを返す
  if((z_far - z_near) - mingap > FLT_EPSILON)
    return true;

  //! 上記以外の場合、falseを返す
  return false;
}

/*!
 *  @brief  ギャップ輪郭線で分離
 *  @param  [in] dimg  dimg
 *  @param  [in] gap  gap
 *  @param  [out] dtmp  dtmp
 *  @param  [in] gappixval  gappixval
 *  @note       N/A
 *  @attention  N/A
 */
static void
separate_gap_contours(cv::Mat &dimg,
                      const double gap,
                      cv::Mat &dtmp,
                      const int gappixval)
{
  dtmp = dimg.clone();

  assert(dimg.isContinuous() && dtmp.isContinuous());

  cv::Size dsize = dimg.size();
  int wstep = dsize.width;

  float *ppd = dimg.ptr<float>();
  float *ppt = dtmp.ptr<float>();

  for(int row=1; row<dsize.height-1; row++)
  {
    int iindex = row*dsize.width + 1;
    float *pd = &ppd[iindex];
    float *pt = &ppt[iindex];

    for(int col=1; col<dsize.width-1; col++)
    {
      if((*pd) > FLT_EPSILON)
      {
        double z1;
        z1 = *pd;
        if(gap_p(z1, *(pd-1), gap) ||
           gap_p(z1, *(pd+1), gap) ||
           gap_p(z1, *(pd-wstep-1), gap) ||
           gap_p(z1, *(pd-wstep), gap) ||
           gap_p(z1, *(pd-wstep+1), gap) ||
           gap_p(z1, *(pd+wstep-1), gap) ||
           gap_p(z1, *(pd+wstep), gap) ||
           gap_p(z1, *(pd+wstep+1), gap))
        {
          //! 8近傍にgap以上離れた点があれば分離
          *pt = gappixval;
        }
      }
      pd++;
      pt++;
    }
  }

  return;
}

/*!
 *  @brief  blob初期化
 *  @param  [out] tb  初期化対象のblob
 *  @note
 *    - blobを初期化する
 *  @attention  N/A
 */
static void
init_blob(eSRS::Blob *tb)
{
  tb->flg = 1; //! flgを 1(active)で初期化する

  tb->c = 0;
  tb->r = 0;
  
  tb->colmin = TC_MAXINT;
  tb->colmax = TC_MININT;
  tb->rowmin = TC_MAXINT;
  tb->rowmax = TC_MININT;
  
  tb->x = 0.0;
  tb->y = 0.0;
  tb->z = 0.0;
  
  tb->xmax = TC_MINDOUBLE;
  tb->ymax = TC_MINDOUBLE;
  tb->zmax = TC_MINDOUBLE;
  tb->xmin = TC_MAXDOUBLE;
  tb->ymin = TC_MAXDOUBLE;
  tb->zmin = TC_MAXDOUBLE;
  
  tb->size = 0;
  return;
}

/*!
 *  @brief  blob更新
 *  @param  [out] tb  更新対象のblob
 *  @param  [in] zval  zval
 *  @param  [in] col  col
 *  @param  [in] row  row
 *  @param  [in] xval  xval
 *  @param  [in] yval  yval
 *  @note
 *    - blobを更新する
 *  @attention  N/A
 */
static void
update_blob(eSRS::Blob *tb, 
            const float zval, const int col, const int row,
            const float xval,
            const float yval)
{
  if(zval < FLT_EPSILON) return;

  tb->c += col;
  tb->r += row;

  if(col < tb->colmin) tb->colmin = col;
  if(col > tb->colmax) tb->colmax = col;
  if(row < tb->rowmin) tb->rowmin = row;
  if(row > tb->rowmax) tb->rowmax = row;

  tb->x += xval;
  tb->y += yval;
  tb->z += zval;

  if(xval > tb->xmax) tb->xmax = xval;
  if(xval < tb->xmin) tb->xmin = xval;
  if(yval > tb->ymax) tb->ymax = yval;
  if(yval < tb->ymin) tb->ymin = yval;
  if(zval > tb->zmax) tb->zmax = zval;
  if(zval < tb->zmin) tb->zmin = zval;

  (tb->size)++;
  
  return;
}

class IdenticalLabel
{
public:
  IdenticalLabel(void) : n(0), min_curr(TC_MAXINT), min_next(TC_MAXINT),
                         flg_curr(true), flg_next(false) {};
  ~IdenticalLabel(void) {};

public:
  void push(const int lbl) { m_searched.push_back(lbl); };

  bool empty(void) { return m_searched.empty(); };

  int search_min(void);
  void init_min(void);

  bool is_updated(void) { return flg_curr; };

  int set_min_next(const int lbl);
  int update_min_next(IdenticalLabel *iregion);
  void copy_min_next(void);

  void print(std::string indent="");

  bool have(const int lbl) {
    return (std::find(m_searched.begin(), m_searched.end(), lbl)
            != m_searched.end());
  };
  
public:
  int n;
  int min_curr;
  int min_next;
  bool flg_curr;
  bool flg_next;

private:
  std::vector<int> m_searched;
};

static short
get_blob_connection(const short *ip, const int wstep,
                    IdenticalLabel *iregion)
{
  short imin=TC_MAXSHORT; // 着目点に隣接している最小のラベル

  short ibuf[4]; // 隣接点4つ
  ibuf[0] = *(ip-1);          //  |1|2|3|
  ibuf[1] = *(ip-wstep-1);    //  |0|*| |
  ibuf[2] = *(ip-wstep);      //  | | | |
  ibuf[3] = *(ip-wstep+1);

  int icount=0;
  for(int i=0; i<4; i++)
  {
    if(ibuf[i] > 0)
    {
      icount++;
      if(imin > ibuf[i])
      {
        imin = ibuf[i];
      }
    }
  }

  if(icount > 1)
  {
    for(int i=0; i<4; i++)
    {
      if(ibuf[i]> 0 && ibuf[i] != imin) // 自分自身を入れない
      {
        if(!iregion[imin].have(ibuf[i]))
        {
          iregion[imin].push(ibuf[i]);
        }
        if(!iregion[ibuf[i]].have(imin))
        {
          iregion[ibuf[i]].push(imin);
        }
      }
    }
  }

  if(imin == TC_MAXSHORT)
    return 0;

  return imin;
}

int
IdenticalLabel::search_min(void)
{
  int min_label = TC_MAXINT;
  for(auto itr=m_searched.begin(); itr != m_searched.end(); ++itr)
  {
    if(min_label > *itr) min_label = *itr;
  }
  
  return min_label;
}

static void
get_identical_label(int identical_label[],
                    IdenticalLabel *iregion,
                    const int count)
{
  assert(iregion[0].empty()); // ラベル0は空のはず

  for(int i=1; i<=count; i++)
  {
    if(!iregion[i].empty())
    {
      int min_label = iregion[i].min_curr;
      
      identical_label[i] = (min_label < i) ? min_label : 0;
    }
  }
    
  return;
}

void
IdenticalLabel::print(std::string indent)
{
  std::cout << indent << this->n << ": ";
  for(auto itr=this->m_searched.begin(); itr != this->m_searched.end(); ++itr)
  {
    std::cout << *itr << ", ";
  }
  std::cout << std::endl;
  
  return;
}

void
IdenticalLabel::init_min(void)
{
  this->min_curr = this->search_min();
  this->min_next = TC_MAXINT;

  this->flg_curr = true;
  this->flg_next = false;
  
  return;
}

void
IdenticalLabel::copy_min_next(void)
{
  if(this->flg_next)
  {
    this->min_curr = this->min_next;
    this->flg_curr = true;

    this->min_next = TC_MAXINT;
    this->flg_next = false;
  }
  
  return;
}

int
IdenticalLabel::set_min_next(const int lbl)
{
  if(this->flg_next == false && this->min_next > lbl)
  {
#ifdef DEBUG1  
    //    std::cout << " (" << this->n << "->" << lbl << ")";
#endif
    this->min_next = lbl;
    this->flg_next = true;
    return 1;
  }
  
  return 0;
}

int
IdenticalLabel::update_min_next(IdenticalLabel *iregion)
{
  int tcount = 0;
  for(auto itr=this->m_searched.begin(); itr != this->m_searched.end(); ++itr)
  {
    if(iregion[*itr].min_curr > this->min_curr)
    {
      tcount += iregion[*itr].set_min_next(this->min_curr);
    }
    if(iregion[*itr].min_curr < this->min_curr)
    {
      //tcount += this->set_min_next(iregion[*itr].min_curr);
    }
  }

  return tcount;
}

static void
set_min_label(IdenticalLabel *iregion, const int count)
{
  for(int i=1; i<=count; i++)
  {
    iregion[i].push(i);    // 自分自身を入れる
    iregion[i].init_min(); // 最小値の初期値をセット
  }

  int scount = 0;
  for(;;)
  {
#ifdef DEBUG1  
    std::cout << "####" << scount << std::endl;
#endif
    int tcount = 0;
    for(int i=1; i<=count; i++)
    {
      if(iregion[i].is_updated())
      {
        tcount += iregion[i].update_min_next(iregion);
      }
    }
    
#ifdef DEBUG1
    for(int i=1; i<=50; i++)
    {
      std::cout << iregion[i].min_curr << ", ";
    }
    std::cout << std::endl;
#endif
    scount++;
    
    for(int i=1; i<=count; i++)
    {
      iregion[i].copy_min_next();
    }
    
    if(tcount == 0) break;
  }

  return;
}

/*!
 *  @brief  label_1st_label
 *  @param  [in] fore  fore
 *  @param  [out] iblob  iblob
 *  @param  [out] iregion  iregion
 *  @param  [out] labeled  labeled
 *  @note       N/A
 *  @attention  N/A
 */
static int
label_1st_label(const cv::Mat &fore, // 32F, 背景:0, 前景:正値, NODATA:負値
                // GAP:-100
                int *identical_label,
                eSRS::Blob *iblob, 
                cv::Mat &labeled,
                const cv::Mat &xmap, // 32F, X座標
                const cv::Mat &ymap // 32F, Y座標
                ) // 16Sのゼロ画像, ラベリング結果を上書き
{
  const float *pdimg = fore.ptr<float>();
  short *pbimg = labeled.ptr<short>();
  const float *pximg = xmap.ptr<float>();
  const float *pyimg = ymap.ptr<float>();

  int height = fore.rows;
  int width = fore.cols;
  int swstep = fore.cols;

  IdenticalLabel *iregion = new IdenticalLabel[_MAXIMUM_LABEL];
  for(int i=0; i<_MAXIMUM_LABEL; i++) iregion[i].n = i;
  
  int count = 0;
  init_blob(&iblob[0]);
  for(int row=1; row<height-1; row++)
  {
    const float *pd = &pdimg[swstep*row + 1];
    short *pb = &pbimg[swstep*row + 1];
    const float *px = &pximg[swstep*row + 1];
    const float *py = &pyimg[swstep*row + 1];

    for(int col=1; col<width-1; col++)
    {
      if(*pd > 0)
      {
        int ret = get_blob_connection(pb, swstep, iregion);
        if(ret == 0)
        {
          count++;
          assert(count < _MAXIMUM_LABEL); // TODO
          init_blob(&iblob[count]);
          *pb = count;
        }
        else
        {
          *pb = ret;
        }
        update_blob(&iblob[*pb], *pd, col, row, *px, *py);
      }
      pd++;
      pb++;
      px++;
      py++;
    }
  }

  set_min_label(iregion, count);
  get_identical_label(identical_label, iregion, count);

#ifdef DEBUG1        
  fprintf(stdout, "%d candidates\n", count);
  for(int i=1; i<50; i++)
  {
    std::cout << identical_label[i] << ", ";
  }
  std::cout << std::endl;
#endif

  delete[] iregion;

  return count;
}

/*!
 *  @brief  blobマージ
 *  @param  [out] dst  マージ先のblob
 *  @param  [in] add  マージ元のblob
 *  @note
 *    - blobをマージする
 *  @attention  N/A
 */
static void
merge_blob(eSRS::Blob &dst, const eSRS::Blob &add)
{
#ifdef DEBUG1        
//  fprintf(stderr, " size  %d + %d\n", dst->size, add->size);
//  fprintf(stderr, " Xmin  %f << %f\n", dst->xmin, add->xmin);
//  fprintf(stderr, " Xmax  %f << %f\n", dst->xmax, add->xmax);
#endif

  //! dstに addを加算する
  dst.c += add.c;
  dst.r += add.r;
  dst.x += add.x;
  dst.y += add.y;
  dst.z += add.z;
  dst.size += add.size;

  //! dstの最大最小値を更新
  if(dst.colmax < add.colmax) dst.colmax = add.colmax;
  if(dst.colmin > add.colmin) dst.colmin = add.colmin;
  if(dst.rowmax < add.rowmax) dst.rowmax = add.rowmax;
  if(dst.rowmin > add.rowmin) dst.rowmin = add.rowmin;

  if(dst.xmax < add.xmax) dst.xmax = add.xmax;
  if(dst.ymax < add.ymax) dst.ymax = add.ymax;
  if(dst.zmax < add.zmax) dst.zmax = add.zmax;
  if(dst.xmin > add.xmin) dst.xmin = add.xmin;
  if(dst.ymin > add.ymin) dst.ymin = add.ymin;
  if(dst.zmin > add.zmin) dst.zmin = add.zmin;

  return;
}

/*!
 *  @brief  label_2nd_label
 *  @param  [in,out] iblob  iblob
 *  @param  [in,out] iregion  iregion
 *  @param  [in] tcount  tcount
 *  @note       N/A
 *  @attention  N/A
 */
static int
label_2nd_label(eSRS::Blob *iblob, int *iregion, const int tcount)
{
  int i;
  int count = tcount;

  /*
  for(i=2; i<=tcount; i++)
  {
    if(iregion[i] != 0)
    {
      //std::cerr << iregion[i] << std::endl;
      assert(iregion[i] >=0 && iregion[i] < _MAXIMUM_LABEL);
      if(iregion[iregion[i]] > 0 && iregion[i] > iregion[iregion[i]])
      {
        iregion[i] =  iregion[iregion[i]];
      }
      count--;
    }
  }
  */
  for(i=2; i<=tcount; i++)
  {
    if(iregion[i] != 0)
    {
#ifdef DEBUG2        
      //fprintf(stderr, "M %d << %d\n", iregion[i], i);
#endif
      merge_blob(iblob[iregion[i]], iblob[i]);
      iblob[i].flg = 0;  // inactive
    }
  }

#ifdef DEBUG1        
  fprintf(stdout, "%d candidates\n", count);
#endif

  return count;
}

/*!
 *  @brief  update_labelimage
 *  @param  [in,out] labeled  labeled
 *  @param  [in] iregion  iregion
 *  @note       N/A
 *  @attention  N/A
 */
static void
update_labelimage(cv::Mat &labeled, const int *iregion)
{
  short *plbl = labeled.ptr<short>();
  long len = labeled.cols*labeled.rows;
  for(int i=0; i<len; i++)
  {
    if(*plbl>0)
      if(iregion[*plbl] > 0) *plbl = iregion[*plbl];
    plbl++;
  }
  return;
}

/*!
 *  @brief  ギャップ修復
 *  @param  [in,out] labeled  labeled
 *  @param  [in] gap  gap
 *  @param  [in] fore  fore
 *  @param  [out] iblob  iblob
 *  @param  [in] gapval  gapval
 *  @param  [in] mingap  ギャップの最小値
 *  @param  [in] focal_length  焦点距離
 *  @param  [in] image_center  画像中心座標
 *  @note       N/A
 *  @attention  N/A
 */
static void
repair_gap(cv::Mat &labeled, 
           cv::Mat &gap,
           cv::Mat &fore,
           eSRS::Blob *iblob,
           const float gapval,
           const double mingap,
           cv::Mat &xmap,
           cv::Mat &ymap)
{
  int width = labeled.cols;
  int height = labeled.rows;

  cv::Mat lclone = labeled.clone();
  
  short *plbldst = labeled.ptr<short>();
  short *plbl = lclone.ptr<short>();
  float *pgap = gap.ptr<float>();
  float *pfore = fore.ptr<float>();
  float *px = xmap.ptr<float>();
  float *py = ymap.ptr<float>();

  const int eight[8] = {-1, -width-1, -width, -width+1,
                        1, width+1, width, width-1};

  for(int i=1; i<height-1; i++)
  {
    for(int j=1; j<width-1; j++)
    {
      int ij = i*width+j;
      if(fabs(pgap[ij] - gapval) < FLT_EPSILON) // GAPである
      {
        if(pfore[ij] > FLT_EPSILON) // /=0, 背景ではない
        {
          int countblob=0;
          int nearestblob=0;
          double mindist=TC_MAXDOUBLE;
          for(int k=0; k<8; k++)
          {
            int ijk = ij+eight[k];
            if(plbl[ijk] != 0) // 8近傍にblobがある
            {
              double curdist = pfore[ijk] - pfore[ij];
              if(curdist < mindist) // それは一番近いのか?
              {
                nearestblob = plbl[ijk];
                mindist = curdist;
              }
              
              int l=0;
              for(; l<k; l++) // 初出かどうか
              {
                if(plbl[ijk] == plbl[ij+eight[l]])  // 既出
                  break;
              }
              if(l==k) countblob++;
            }
          }

          if(countblob > 1 && // 2個以上のblobと接している(1個なら孤立点)
             nearestblob != 0 && // 最近傍 blobが存在して
             mindist - mingap < FLT_EPSILON) // 閾値より近い
          {
            plbldst[ij] = nearestblob;
            update_blob(&iblob[nearestblob],
                        pfore[ij],
                        j, i,
                        px[ij],
                        py[ij]);
          }
        }
      }
    }
  }

  return;
}

/*!
 *  @brief  obstacle情報作成
 *  @param  [out] obsvec  obstacle情報
 *  @param  [out] iregion  iregion
 *  @param  [in] iblob  iblob
 *  @param  [in] fcount  fcount
 *  @param  [in] min_size  最小サイズ
 *  @note       N/A
 *  @attention  N/A
 */
static int
make_obstacle(std::vector<eSRS::Obstacle> &obsvec,
              int *iregion,
              const eSRS::Blob *iblob,
              const int fcount,
              const int min_size)
{
  int ocount=0;
  for(int i=1; i<=fcount; i++)
  {
    iregion[i] = 0;
    if(iblob[i].flg)
    {
      if(iblob[i].size >= min_size)
      {
        eSRS::Obstacle obs;
        obs.n = ocount;
        obs.bbox.x = iblob[i].colmin;
        obs.bbox.y = iblob[i].rowmin;
        obs.bbox.width = iblob[i].colmax - iblob[i].colmin + 1;
        obs.bbox.height = iblob[i].rowmax - iblob[i].rowmin + 1;
        obs.bvol.x = iblob[i].xmin;
        obs.bvol.y = iblob[i].ymin;
        obs.bvol.z = iblob[i].zmin;
        obs.bvol.width = iblob[i].xmax - iblob[i].xmin;
        obs.bvol.height = iblob[i].ymax - iblob[i].ymin;
        obs.bvol.depth = iblob[i].zmax - iblob[i].zmin;
        assert(iblob[i].size > 0);

        obs.igrv.x = iblob[i].c / iblob[i].size;
        obs.igrv.y = iblob[i].r / iblob[i].size;
        
        obs.grv.x = iblob[i].x / iblob[i].size;
        obs.grv.y = iblob[i].y / iblob[i].size;
        obs.grv.z = iblob[i].z / iblob[i].size;
        obs.size = iblob[i].size;

        obsvec.push_back(obs);

        iregion[i] = ocount+1; // プラス1。ゼロは背景
      
        ocount++;
      }
    }
  }
  
  return ocount;
}

/*!
 *  @brief  obstacleイメージ更新
 *  @param  [in,out] labeled  labeled
 *  @param  [in] iregion  iregion
 *  @note       N/A
 *  @attention  N/A
 */
static void
update_obstacleimage(cv::Mat &labeled, const int *iregion)
{
  short *plbl = labeled.ptr<short>();
  long len = labeled.cols*labeled.rows;
  for(int i=0; i<len; i++)
  {
    if(*plbl>0)
      *plbl = iregion[*plbl];
    plbl++;
  }
  return;
}

int
eSRS::DetectionMap::detect(void)
{
  //! 過去の検出結果をクリアする
  m_obs_vec.clear();

  //! データをコピーする
  DetectionMap dcpy(*this);
  cv::Mat dmap(cv::Size(dcpy.width(), dcpy.height()), CV_32F, dcpy.data());
  cv::Mat xmap(cv::Size(dcpy.width(), dcpy.height()), CV_32F, dcpy.xdata());
  cv::Mat ymap(cv::Size(dcpy.width(), dcpy.height()), CV_32F, dcpy.ydata());
  double maxdepth = this->getMaxDepth();

  //! 奥行きで閾値処理
  cv::Mat fore, bin;
  cv::threshold(dmap, fore, m_zkey, maxdepth-1, cv::THRESH_TOZERO_INV);
  cv::threshold(dmap, bin, m_zkey, maxdepth-1, cv::THRESH_BINARY);
  cv::add(fore, bin, dmap);

#ifdef DEBUG3
  this->display("Depth", -1);
  eSRS::DepthMap fmap(dcpy.width(), dcpy.height(), fore.ptr<float>(),
                      dcpy.getMaxDepth());
//  fmap.display("Fore", 1);
  dcpy.display("Zkey 1", 1);
#endif

  //! ギャップ輪郭線で分離
  cv::Mat sep, sepfore;
  separate_gap_contours(dmap, m_min_gap, sep, 
                        eSRS::DetectionMap::PIXELVALUE_GAP);
  // sepfore : 背景がゼロ
  cv::threshold(sep, sepfore, m_zkey, maxdepth-1, cv::THRESH_TOZERO_INV);
#ifdef DEBUG2
  eSRS::DepthMap smap(dcpy.width(), dcpy.height(), sepfore.ptr<float>(),
                      dcpy.getMaxDepth());
  smap.display("Gap", -1);
#endif

  //! ラベリング
  //int iregion[_MAXIMUM_LABEL]={0}; // TODO : std::vector に変更
  //std::vector<std::vector<int>> iregion[_MAXIMUM_LABEL]; // 同ラベルのリスト
  //int identical_label[_MAXIMUM_LABEL]; // 同ラベルのリスト
  //eSRS::Blob iblob[_MAXIMUM_LABEL]; // TODO : std::vector に変更
  int *identical_label = new int[_MAXIMUM_LABEL];
  eSRS::Blob *iblob = new eSRS::Blob[_MAXIMUM_LABEL]; // TODO : std::vector に変更

  cv::Mat labeled(cv::Size(m_labeled.width(),
                           m_labeled.height()), CV_16S, m_labeled.data());
  labeled.setTo(0.0);
  //cv::Mat labeled = cv::Mat::zeros(dcpy.height(), dcpy.width(), CV_16S);

  int fcount = label_1st_label(sepfore,
                               identical_label,
                               iblob, labeled,
                               xmap, ymap);


#ifdef DEBUG2
  UFV::ImageData<short> limage(dcpy.width(), dcpy.height(), 1,
                                labeled.ptr<short>());
  short *plbl = limage.data();
  for(int i=0; i<dcpy.width()*dcpy.height(); i++)
  {
    if(*plbl == 0)
      *plbl = fcount + 1;
    else if (*plbl % 2)
      *plbl = *plbl + 500;
    
    plbl++;
  }
  limage.display("Label 1", -1, fcount+1);
#endif  

  //! ラベルをマージ
#ifdef DEBUG1
  int scount = label_2nd_label(iblob, identical_label, fcount);
#else
  label_2nd_label(iblob, identical_label, fcount);
#endif
  update_labelimage(labeled, identical_label);

#ifdef DEBUG3
  UFV::ImageData<short> limage2(dcpy.width(), dcpy.height(), 1,
                                 labeled.ptr<short>());
  short *plbl2 = limage2.data();
  for(int i=0; i<dcpy.width()*dcpy.height(); i++)
  {
    if(*plbl2 == 0)
      *plbl2 = fcount + 1;
    plbl2++;
  }
  limage2.display("Label 2", -1, fcount+1);
#endif  

  //! GAP部分を可能な限り修復
  repair_gap(labeled, sepfore, fore, iblob,
             eSRS::DetectionMap::PIXELVALUE_GAP,
             m_min_gap,
             xmap, ymap);

#ifdef DEBUG1
  UFV::ImageData<short> limage3(dcpy.width(), dcpy.height(), 1,
                                 labeled.ptr<short>());
  short *plbl3 = limage3.data();
  for(int i=0; i<dcpy.width()*dcpy.height(); i++)
  {
    if(*plbl3 == 0)
      *plbl3 = fcount + 1;
    plbl3++;
  }
  limage3.display("Label 3", 1, fcount+1);
#endif  

  //! obstacle情報作成
  int ocount = make_obstacle(m_obs_vec, identical_label, iblob, fcount, m_min_size);
  update_obstacleimage(labeled, identical_label);

#ifdef DEBUG1
  fprintf(stdout, "\n%d obstacles (%d noises)\n", ocount, scount-ocount);
  UFV::ImageData<short> limage4(dcpy.width(), dcpy.height(), 1,
                                 labeled.ptr<short>());
  short *plbl4 = limage4.data();
  for(int i=0; i<dcpy.width()*dcpy.height(); i++)
  {
    if(*plbl4 == 0)
      *plbl4 = ocount + 10;
    plbl4++;
  }
  limage4.display("Obstacle", -1, ocount+10);
#endif

  //m_labeled.display("Labeled", -1, 32);

#if defined(DEBUG3)
  FILE *fp;
  fp = fopen("blob6.yml", "w");
  print_obstacles(fp, obs);
  fclose(fp);
#endif

  delete[] identical_label;
  delete[] iblob;

  //! 検出数を返す
  return ocount;
}

// DepthMapと検出結果のコピー
void
eSRS::DetectionMap::normalizeWithBoundingBox(UFV::ImageData<unsigned char> &bbox,
                                             const double maxdepth)
  const
{
  //! コピー先とコピー元(Depth Map)の width、height、チェンネル数のいずれかが異なる場合、
  //! コピー先をコピー元(Depth Map)のサイズで reshapeする
  if(bbox.width() != this->width() || bbox.height() != this->height() ||
     bbox.nchannels() != 3)
  {
    bbox.reshape(this->width(), this->height(), 3);
  }

  //! Depth Mapを bboxにコピーする
  this->normalize(bbox, maxdepth);

  this->drawBoundingBox(bbox);

  /*
  //! 検出結果を表す矩形の枠線の太さを設定する
  int lwidth=2;
  if(bbox.width() <= 320 || bbox.height() <= 240) lwidth = 1;

  //! 検出結果描画用の Matインスタンスを生成する
  cv::Mat img(cv::Size(this->width(),this->height()), CV_8UC3, bbox.data());

  //! 検出結果数分、以下の処理を行う
  cv::Point p1, p2;
  for(unsigned int i=0; i<m_obs_vec.size(); i++)
  {
    //! - 矩形の頂点と反対側の頂点を算出する
    p1.x = m_obs_vec[i].bbox.x;
    p1.y = m_obs_vec[i].bbox.y;
    p2.x = m_obs_vec[i].bbox.x + m_obs_vec[i].bbox.width - 1;
    p2.y = m_obs_vec[i].bbox.y + m_obs_vec[i].bbox.height - 1;

    //! - 画像に矩形を描画する(枠線の色は赤)
    cv::rectangle(img, p1, p2, cv::Scalar(0,0,255), lwidth, 8);
  }
  */
  return;
}

// DepthMapと検出結果のコピー
void
eSRS::DetectionMap::normalizeWithObstacleRegion(UFV::ImageData<unsigned char> &bbox,
  const double maxdepth)
  const
{
  //! コピー先とコピー元(Depth Map)の width、height、チェンネル数のいずれかが異なる場合、
  //! コピー先をコピー元(Depth Map)のサイズで reshapeする
  if (bbox.width() != this->width() || bbox.height() != this->height() ||
    bbox.nchannels() != 3)
  {
    bbox.reshape(this->width(), this->height(), 3);
  }

  //! Depth Mapを bboxにコピーする
  this->normalize(bbox, maxdepth);

  this->drawObstacleRegionWithLabel(bbox);

  return;
}

static void
get_xyz_label(const eSRS::BoundingVolume vol,
              const double fscale,
              const int face,
              const int thickness,
              const int lwidth,
              const cv::Point p1,
              const cv::Point p2,
              std::string &label,
              cv::Point &rectp1, cv::Point &rectp2, cv::Point &textp)
{
  //label = std::to_string((int)std::round(vol.z));
   label = std::to_string((int)std::round(vol.z)) + ","
    + std::to_string((int)std::round(vol.width)) + "x"
    + std::to_string((int)std::round(vol.height));

  //std::cout << ">>> " << label << std::endl;

  int baseline = 0;
  cv::Size tsize = cv::getTextSize(label,
                                   face, fscale, thickness,
                                   &baseline);
  int labelheight = tsize.height+baseline+lwidth/2;
  int labelhwidth = (tsize.width)/2;
  cv::Point c1 = cv::Point((p1.x+p2.x)/2, (p1.y+p2.y)/2);

  if(labelheight > c1.y)
  {
    rectp1 = c1 + cv::Point(-labelhwidth, labelheight);
    rectp2 = c1 + cv::Point(labelhwidth, 0);
    textp = c1 + cv::Point(-labelhwidth, labelheight - baseline);
  }
  else
  {
    rectp1 = c1 + cv::Point(-labelhwidth, (labelheight)/2);
    rectp2 = c1 + cv::Point(labelhwidth, -labelheight/2);
    textp = c1 + cv::Point(-labelhwidth, -baseline + labelheight/2);
  }
  
  return;
}


// 検出結果描画
int
eSRS::DetectionMap::drawBoundingBox(UFV::ImageData<unsigned char> &bbox)
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

    //! - 画像に矩形を描画する(枠線の色は赤)
    cv::rectangle(img, p1, p2, cv::Scalar(0,0,255), lwidth, 8);

    if(this->m_flg_drawlabel)
    {
      //double fscale = 0.5/mag;
      double fscale = 0.5;
      int face = cv::FONT_HERSHEY_SIMPLEX;
      int thickness = lwidth/3;
      std::string xyzlabel;
      cv::Point rectp1, rectp2, textp;
      get_xyz_label(m_obs_vec[i].bvol, fscale, face, thickness, lwidth,
                    p1, p2,
                    xyzlabel, rectp1, rectp2, textp);
      cv::rectangle(img, rectp1, rectp2, cv::Scalar(0,0,255), cv::FILLED, 8);
      cv::putText(img, xyzlabel.c_str(), textp, face, fscale, cv::Scalar(0,0,0),
                  thickness, cv::LINE_AA);
    }
  }

  //this->printObstacles(stdout);

  return UFV::OK;
}

// 検出結果描画(オフセット指定)
int
eSRS::DetectionMap::drawBoundingBox(UFV::ImageData<unsigned char> &bbox,
                                    const UFV::Point2Di &offset)
  const
{

  //! 検出結果描画用の Matインスタンスを生成する
  cv::Mat img(cv::Size(bbox.width(),bbox.height()), CV_8UC3, bbox.data());

  //! 検出結果を表す矩形の枠線の太さを設定する
  int lwidth=2;
  if(bbox.width() <= 320 || bbox.height() <= 240) lwidth = 1;

  cv::Point p1, p2;

  //! 検出結果数分、以下の処理を行う
  for(unsigned int i=0; i<m_obs_vec.size(); i++)
  {
    //! - オフセット値を使用し、矩形の頂点と反対側の頂点を算出する
    p1.x = m_obs_vec[i].bbox.x + offset.x;
    p1.y = m_obs_vec[i].bbox.y + offset.y;
    p2.x = m_obs_vec[i].bbox.x + offset.x + m_obs_vec[i].bbox.width - 1;
    p2.y = m_obs_vec[i].bbox.y + offset.y + m_obs_vec[i].bbox.height - 1;

    //! - 画像に矩形を描画する(枠線の色は赤)
    cv::rectangle(img, p1, p2, cv::Scalar(0,0,255), lwidth, 8);
  }

  return UFV::OK;
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

  labelwidth += lwidth * 2;
  //int labelheight1 = tsize_cls.height+baseline1+lwidth/2;
  //int labelheight2 = tsize_xyz.height+baseline2+lwidth/2;
  int labelheight1 = tsize_cls.height + lwidth + baseline1;
  int labelheight2 = tsize_xyz.height + lwidth;
  int labelhwidth = labelwidth / 2;

  rectp1 = c1 + cv::Point(-labelhwidth, labelheight2);
  rectp2 = c1 + cv::Point(labelhwidth, -labelheight1);
  //textp_cls = c1 + cv::Point(-labelhwidth+lwidth, -baseline1);
  //textp_xyz = c1 + cv::Point(-labelhwidth+lwidth, -lwidth + labelheight2);
  textp_cls = c1 + cv::Point(-tsize_cls.width / 2, -baseline1);
  textp_xyz = c1 + cv::Point(-tsize_xyz.width / 2, -lwidth + labelheight2);

  return;
}

int
eSRS::DetectionMap::drawObstacleRegion(UFV::ImageData<unsigned char> &bbox)
const
{
  //! 検出結果描画先と Depth Mapの width、height、チャンネル数の
  //  いずれかが異なる場合、エラー復帰する
  if (bbox.width() != this->width() || bbox.height() != this->height() ||
    bbox.nchannels() != 3)
  {
    return UFV::NG;
  }

  //! 検出結果描画用の Matインスタンスを生成する
  cv::Mat img(cv::Size(this->width(), this->height()), CV_8UC3, bbox.data());

  UFV::Color ucolor(255.0, 0.0, 0.0);

  short *plbl = m_labeled.data();
  uchar *pbox = bbox.data();
  for (int i = 0; i < this->width()*this->height(); i++)
  {
    if (*plbl > 0)
    {
        *pbox *= ucolor.b / 255.0;
        *(pbox + 1) *= ucolor.g / 255.0;
        *(pbox + 2) *= ucolor.r / 255.0;
    }
    pbox += 3;
    plbl++;
  }

#ifdef DRAWLABELBOX
  //! 検出結果を表す矩形の枠線の太さを設定する
  int lwidth = bbox.height() * 0.005;

  //! 検出結果数分、以下の処理を行う
  for (unsigned int i = 0; i < m_obs_vec.size(); i++)
  {
    //! - 矩形の頂点と反対側の頂点を算出する
    cv::Point p1;
    p1.x = m_obs_vec[i].igrv.x;
    p1.y = m_obs_vec[i].igrv.y;

    //! - 画像に矩形を描画する
    if (this->m_flg_drawlabel)
    {
      //double fscale = 0.5/mag;
      double fscale1 = 0.5;
      double fscale2 = 0.35;
      int face = cv::FONT_HERSHEY_SIMPLEX;
      int thickness = lwidth / 3;
      std::string xyzlabel;
      cv::Point rectp1, rectp2, textp_xyz, textp_cls;
      get_reg_label(m_obs_vec[i].bvol, fscale1, fscale2,
        face, thickness, lwidth,
        p1, "unknown",
        xyzlabel, rectp1, rectp2,
        textp_cls, textp_xyz);

      cv::rectangle(img, rectp1, rectp2,
        cv::Scalar(ucolor.b, ucolor.g, ucolor.r),
        cv::FILLED, 8);
      cv::putText(img, "unknown",
        textp_cls, face, fscale1,
        cv::Scalar(0, 0, 0), thickness, cv::LINE_AA);
      cv::putText(img, xyzlabel.c_str(),
        textp_xyz, face, fscale2,
        cv::Scalar(0, 0, 0), thickness, cv::LINE_AA);

    }
  }
#endif

  //this->printObstacles(stdout);

  return UFV::OK;
}

int
eSRS::DetectionMap::drawObstacleRegionWithLabel(UFV::ImageData<unsigned char> &bbox)
const
{
  //! 検出結果描画先と Depth Mapの width、height、チャンネル数の
  //  いずれかが異なる場合、エラー復帰する
  if (bbox.width() != this->width() || bbox.height() != this->height() ||
    bbox.nchannels() != 3)
  {
    return UFV::NG;
  }

  //! 検出結果描画用の Matインスタンスを生成する
  cv::Mat img(cv::Size(this->width(), this->height()), CV_8UC3, bbox.data());

  UFV::Color ucolor(255.0, 0.0, 0.0);

  short *plbl = m_labeled.data();
  uchar *pbox = bbox.data();
  for (int i = 0; i < this->width()*this->height(); i++)
  {
    if (*plbl > 0)
    {
      *pbox *= ucolor.b / 255.0;
      *(pbox + 1) *= ucolor.g / 255.0;
      *(pbox + 2) *= ucolor.r / 255.0;
    }
    pbox += 3;
    plbl++;
  }

  //! 検出結果を表す矩形の枠線の太さを設定する
  int lwidth = bbox.height() * 0.005;

  //! 検出結果数分、以下の処理を行う
  for (unsigned int i = 0; i < m_obs_vec.size(); i++)
  {
    //! - 矩形の頂点と反対側の頂点を算出する
    cv::Point p1;
    p1.x = m_obs_vec[i].igrv.x;
    p1.y = m_obs_vec[i].igrv.y;

    //! - 画像に矩形を描画する
    if (this->m_flg_drawlabel)
    {
      //double fscale = 0.5/mag;
      double fscale1 = 0.5;
      double fscale2 = 0.35;
      int face = cv::FONT_HERSHEY_SIMPLEX;
      int thickness = lwidth / 3;
      std::string xyzlabel;
      cv::Point rectp1, rectp2, textp_xyz, textp_cls;
      get_reg_label(m_obs_vec[i].bvol, fscale1, fscale2,
        face, thickness, lwidth,
        p1, "unknown",
        xyzlabel, rectp1, rectp2,
        textp_cls, textp_xyz);

      cv::rectangle(img, rectp1, rectp2,
        cv::Scalar(ucolor.b, ucolor.g, ucolor.r),
        cv::FILLED, 8);
      cv::putText(img, "unknown",
        textp_cls, face, fscale1,
        cv::Scalar(0, 0, 0), thickness, cv::LINE_AA);
      cv::putText(img, xyzlabel.c_str(),
        textp_xyz, face, fscale2,
        cv::Scalar(0, 0, 0), thickness, cv::LINE_AA);

    }
  }

  //this->printObstacles(stdout);

  return UFV::OK;
}

// 画像表示(表示範囲指定)
int
eSRS::DetectionMap::display(const std::string wlabel, const int msec,
                            const UFV::Rect &rect,
                            const double mag,
                            const double maxdepth) const
{
  //! ImageDataインスタンスを生成する
  UFV::ImageData<unsigned char> showimg;

  //! ImageDataインスタンスに DepthMapと検出結果をコピーする
  this->normalizeWithBoundingBox(showimg, maxdepth);

  //! ImageDataインスタンスを元に、画像表示用の Matインスタンスを生成する
  cv::Mat simg(cv::Size(this->width(), this->height()), CV_8UC3, 
               showimg.data());

  //! 表示範囲を生成する
  cv::Rect roi(rect.x, rect.y, rect.width, rect.height);

  //! 表示倍率が(TODO:条件の詳細)の場合、そのままのサイズで画像を表示する
  if(mag<FLT_EPSILON || fabs(mag-1.0) < FLT_EPSILON)
    cv::imshow(wlabel, simg(roi));

  //! 表示倍率が上記以外の場合、最近傍補間でリサイズし画像を表示する
  else
  {
    cv::Mat dst;
    cv::resize(simg(roi), dst, cv::Size(), mag, mag, cv::INTER_NEAREST);
    cv::imshow(wlabel, dst);
  }

  //! タイムアウト値が 0以上の場合、cv::waitKey(msec)にてキー入力を待ち受ける
  if(msec >= 0)
  {
    int rkey = cv::waitKey(msec);

    //! - 'q'のキー入力を検出した場合、END_OF_FILEで復帰する
    if(rkey == XK_q)
    return UFV::END_OF_FILE;

    //! - SPACEのキー入力を検出した場合、次のキー入力を待ち続ける
    else if(rkey == XK_space)
      cv::waitKey(0);
  }

  return UFV::OK;
}

// 画像表示
int
eSRS::DetectionMap::display(const std::string wlabel, const int msec,
                            const double maxdepth) const
{
  //! ImageDataインスタンスを生成する
  UFV::ImageData<unsigned char> showimg;

  //! ImageDataインスタンスに DepthMapと検出結果をコピーする
  this->normalizeWithBoundingBox(showimg, maxdepth);

  //! ImageDataインスタンスを元に、画像表示用の Matインスタンスを生成する
  cv::Mat simg(cv::Size(this->width(), this->height()), CV_8UC3, 
               showimg.data());

  //! 画像を表示する
  cv::imshow(wlabel, simg);
  
  //! タイムアウト値が 0以上の場合、cv::waitKey(msec)にてキー入力を待ち受ける
  if(msec >= 0)
  {
    int rkey = cv::waitKey(msec);

    //! - 'q'のキー入力を検出した場合、END_OF_FILEで復帰する
    if(rkey == XK_q)
      return UFV::END_OF_FILE;
    
    //! - SPACEのキー入力を検出した場合、次のキー入力を待ち続ける
    else if(rkey == XK_space)
      cv::waitKey(0);
  }

  return UFV::OK;
}

// 画像ファイル書き込み
void
eSRS::DetectionMap::writeImage(const std::string name,
                               const double maxdepth) const
{
  //! 画像ファイル書き込みフラグが true以外の場合、処理をせずに復帰する
  if(this->getWriteImage() != true) return;

  //! ImageDataインスタンスを生成する
  UFV::ImageData<unsigned char> showimg;

  //! ImageDataインスタンスに DepthMapと検出結果をコピーする
  this->normalizeWithBoundingBox(showimg, maxdepth);

  //! ImageDataインスタンスを元に、画像ファイル書き込み用の Matインスタンスを生成する
  cv::Mat simg(cv::Size(this->width(), this->height()), CV_8UC3, 
               showimg.data());

  std::string file_name;

  //! ファイル名が引数で指定された場合、指定された名前でファイルパス名を作成する
  if(name != ""){
    file_name = this->getWriteDirectory() + name;

  //! ファイル名が引数で指定されていない場合、現在時刻＋".bmp"でファイルパス名を作成する
  }else{
    file_name = this->getWriteDirectory() + UFV::getLocalTimeString() + ".bmp";
  }

  //! 画像データをファイルに書き込む
  cv::imwrite(file_name, simg);

  return;
}

// 画像ファイル書き込み(表示範囲・倍率指定)
void
eSRS::DetectionMap::writeImage(const UFV::Rect &rect, const double mag,
                               const std::string name,
                               const double maxdepth) const
{
  //! 画像ファイル書き込みフラグが true以外の場合、処理をせずに復帰する
  if(this->getWriteImage() != true) return;

  //! ImageDataインスタンスを生成する
  UFV::ImageData<unsigned char> showimg;

  //! ImageDataインスタンスに DepthMapと検出結果をコピーする
  this->normalizeWithBoundingBox(showimg, maxdepth);

  //! ImageDataインスタンスを元に、画像ファイル書き込み用の Matインスタンスを生成する
  cv::Mat simg(cv::Size(this->width(), this->height()), CV_8UC3, 
               showimg.data());

  //! 表示範囲を生成する
  cv::Rect roi(rect.x, rect.y, rect.width, rect.height);

  std::string file_name;

  //! ファイル名が引数で指定された場合、指定された名前でファイルパス名を作成する
  if(name != ""){
    file_name = this->getWriteDirectory() + name;

  //! ファイル名が引数で指定されていない場合、現在時刻＋".bmp"でファイルパス名を作成する
  }else{
    file_name = this->getWriteDirectory() + UFV::getLocalTimeString() + ".bmp";
  }

  //! 表示倍率が(TODO:条件の詳細)の場合、そのままのサイズで画像データをファイルに書き込む
  if(mag<FLT_EPSILON || fabs(mag-1.0) < FLT_EPSILON)
    cv::imwrite(file_name, simg(roi));

  //! 表示倍率が上記以外の場合、最近傍補間でリサイズし画像データをファイルに書き込む
  else
  {
    cv::Mat dst;
    cv::resize(simg(roi), dst, cv::Size(), mag, mag, cv::INTER_NEAREST);
    cv::imwrite(file_name, dst);
  }

  return;
}

void
eSRS::DetectionMap::printObstacles(FILE *fp) const
{
  fprintf(fp, "---\n");
  for (unsigned int i = 0; i<m_obs_vec.size(); i++)
  {
    fprintf(fp, "- N: %d\n", i);
    
    fprintf(fp, "  ORG: [%f, %f, %f]\n",
            m_obs_vec[i].bvol.x, m_obs_vec[i].bvol.y, m_obs_vec[i].bvol.z);

    fprintf(fp, "  DIM: [%f, %f, %f]\n",
            m_obs_vec[i].bvol.width, m_obs_vec[i].bvol.height,
            m_obs_vec[i].bvol.depth);

    fprintf(fp, "  GRV: [%f, %f, %f]\n",
            m_obs_vec[i].grv.x, m_obs_vec[i].grv.y, m_obs_vec[i].grv.z);

    fprintf(fp, "  IGRV: [%f, %f]\n",
            m_obs_vec[i].igrv.x, m_obs_vec[i].igrv.y);
  }

  return;
}
