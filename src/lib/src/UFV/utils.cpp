/**
 * @file utils.cpp
 *
 * @author Yasushi SUMI (AIST)
 *
 * Copyright (C) 2008  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_WINSOCK2_H
#include <winsock2.h>
#endif
#ifdef HAVE_CONIO_H
#include <conio.h>
#define HAVE_KBHIT
#endif

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ctype.h>
#if HAVE_TIME_H
#include <time.h>
#endif
#if HAVE_SYS_TIME_H
#include <sys/time.h>
#endif
#if HAVE_TERMIOS_H
#include <termios.h>
#endif
#if HAVE_FCNTL_H
#include <fcntl.h>
#endif

#include "UFV/types.h"
#include "UFV/utils.h"

#ifdef HAVE_KBHIT
int
UFV::kbhit(void)
{
  return ::kbhit();
}
#else
int
UFV::kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
  
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  
  ch = getchar();
  
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  
  return 0;
}
#endif

#ifdef HAVE_USLEEP
int
UFV::usleep(const unsigned int usec) 
{ 
  return ::usleep(usec); 
}
#else // HAVE_USLEEP
# ifdef HAVE_COIL_TIME_H
#  include <coil/Time.h>
int
UFV::usleep(const unsigned int usec) 
{ 
  return coil::usleep(usec); 
}
# else //  HAVE_COIL_TIME_H
int
UFV::usleep(const unsigned int usec)
{
#  ifdef _WIN32
  Sleep(usec / 1000);
  return 0;
#  endif
  return -1;
}
# endif //  HAVE_COIL_TIME_H
#endif //  HAVE_USLEEP

/*********************************************************************
 * get time
 */
UFV::Time
UFV::getTime(void)
{
  UFV::Time ufvtime;

#ifdef _WIN32
/*
  DWORD a;
  a = timeGetTime();  // milliseconds

  ufvtime.sec = a / 1000;
  ufvtime.usec = (a - ufvtime.sec * 1000) * 1000; // microseconds
*/

  SYSTEMTIME syst;
  struct tm st;
  
  GetLocalTime(&syst);  // 遅いか？

  st.tm_year = syst.wYear - 1900;
  st.tm_mon = syst.wMonth - 1;
  st.tm_mday = syst.wDay;
  st.tm_hour = syst.wHour;
  st.tm_min = syst.wMinute;
  st.tm_sec = syst.wSecond;
  //st.tm_usec = syst.wMilliseconds * 1000;
  st.tm_isdst = -1;
  st.tm_wday = 0;
  st.tm_yday = 0;
  
  ufvtime.sec = mktime(&st);
  ufvtime.usec = syst.wMilliseconds * 1000;

#else
  struct timeval a;
  gettimeofday(&a, NULL);

  ufvtime.sec = a.tv_sec;
  ufvtime.usec = a.tv_usec;
#endif

/*
  std::cout << "ATime" << " (" << ufvtime.sec << ", " 
            << ufvtime.usec << ")  " << std::endl;
*/

  return ufvtime;
}

UFV::Time
UFV::getTime(const UFV::LocalTime &ltm)
{
  UFV::Time ufvtime;

  struct tm st;
  
  st.tm_year = ltm.year;
  st.tm_mon = ltm.month - 1;
  st.tm_mday = ltm.mday;
  st.tm_hour = ltm.hour;
  st.tm_min = ltm.minute;
  st.tm_sec = ltm.second;
  //st.tm_usec = syst.wMilliseconds * 1000;
  st.tm_isdst = -1;
  st.tm_wday = 0;
  st.tm_yday = 0;
  
  ufvtime.sec = mktime(&st);
  ufvtime.usec = ltm.millisecond * 1000;

  return ufvtime;
}

double
UFV::calcLaptime(const UFV::Time &fromtime, const UFV::Time &totime)
{
  double from_msec = fromtime.sec * 1000.0 + fromtime.usec / 1000.0;
  double to_msec = totime.sec * 1000.0 + totime.usec / 1000.0;
  return to_msec - from_msec;
}

void
UFV::LapTimer::print(const char *indent, const char *eol)
{
  UFV::Time ctime = UFV::getTime();
  std::cout << indent
            << std::setw(8) 
            << UFV::calcLaptime(m_time, ctime)
            << eol
            << std::flush;
    //<< " #" << m_count
    //        << std::endl;
  m_time = ctime;
  m_count++;
  return;
}

void
UFV::LapTimer::print(std::ostream &os, const char *indent, const char *eol)
{
  UFV::Time ctime = UFV::getTime();
  os << indent
     << std::setw(8) 
     << UFV::calcLaptime(m_time, ctime)
     << eol
     << std::flush;
  //<< " #" << m_count;
  m_time = ctime;
  m_count++;
  return;
}

double
UFV::LapTimer::get(void)
{
  UFV::Time ctime = UFV::getTime();
  double ret = UFV::calcLaptime(m_time, ctime);
  m_time = ctime;
  m_count++;
  return ret;
}

std::string
UFV::getLocalTimeString(const UFV::LocalTime &ltm)
{
  std::stringstream str_stream;
  str_stream << std::setw(2) << std::setfill('0') << ltm.year
             << std::setw(2) << std::setfill('0') << ltm.month
             << std::setw(2) << std::setfill('0') << ltm.mday
             << std::setw(1) << "_"
             << std::setw(2) << std::setfill('0') << ltm.hour
             << std::setw(2) << std::setfill('0') << ltm.minute
             << std::setw(1) << "_"
             << std::setw(2) << std::setfill('0') << ltm.second
             << std::setw(3) << std::setfill('0') << ltm.millisecond;
  return str_stream.str();
}

std::string
UFV::getLocalTimeString(void)
{
  UFV::LocalTime ltm = UFV::getLocalTime();
  return getLocalTimeString(ltm);
}


UFV::LocalTime
UFV::getLocalTime(void)
{
  UFV::LocalTime t;

#ifdef _WIN32  
  SYSTEMTIME st;
  GetLocalTime(&st);

  t.year = st.wYear;
  t.month = st.wMonth;
  t.mday = st.wDay;
  t.hour = st.wHour;
  t.minute = st.wMinute;
  t.second = st.wSecond;
  t.millisecond = st.wMilliseconds;
#else
  struct timeval a;
  struct tm st;

  gettimeofday(&a, NULL);
  localtime_r(&a.tv_sec, &st);
  
  t.year = st.tm_year + 1900;
  t.month = st.tm_mon + 1;
  t.mday = st.tm_mday;
  t.hour = st.tm_hour;
  t.minute = st.tm_min;
  t.second = st.tm_sec;
  t.millisecond = a.tv_usec / 1000;
#endif

  return t;
}

UFV::LocalTime
UFV::getLocalTime(const UFV::Time &tm)
{
  UFV::LocalTime ltm;

  time_t a = tm.sec;

  struct tm *st = localtime(&a);

  ltm.year = st->tm_year + 1900;
  ltm.month = st->tm_mon + 1;
  ltm.mday = st->tm_mday;
  ltm.hour = st->tm_hour;
  ltm.minute = st->tm_min;
  ltm.second = st->tm_sec;
  ltm.millisecond = tm.usec / 1000;
  
  return ltm;
}

/**
 * Random number generator.
 */
int
UFV::getRandomNumber(const int rmin, const int rmax)
{
  return rmin + (int) ((rmax - rmin + 1.0) * rand() / (RAND_MAX + 1.0));
}

void
UFV::convRMatToXYZrad(const double *R, double *X, double *Y, double *Z)
{
  if(R[0] == 0.0 && R[3] == 0.0)
  {
    if(X != NULL)
      *X = atan2(-R[5], R[4]);

    if(Y != NULL)
    {
      if (R[6] < 0.0)
        *Y = M_PI / 2.0;
      else
        *Y = 1.5 * M_PI;
    }

    if(Z != NULL)
      *Z = 0.0;
  }
  else
  {
    if(X != NULL)
      *X = atan2(R[7], R[8]);
    if(Y != NULL)
      *Y = -asin(R[6]);
    if(Z != NULL)
      *Z  = atan2(R[3], R[0]);
  }

  return;
}

void
UFV::convHMatToXYZrad(const double *H, double *X, double *Y, double *Z)
{
  if(H[0] == 0.0 && H[4] == 0.0)
  {
    if(X != NULL)
      *X = atan2(-H[6], H[5]);

    if(Y != NULL)
    {
      if (H[8] < 0.0)
        *Y = M_PI / 2.0;
      else
        *Y = 1.5 * M_PI;
    }

    if(Z != NULL)
      *Z = 0.0;
  }
  else
  {
    if(X != NULL)
      *X = atan2(H[9], H[10]);
    if(Y != NULL)
      *Y = -asin(H[8]);
    if(Z != NULL)
      *Z  = atan2(H[4], H[0]);
  }

  return;
}

void
UFV::convXradToRMat(const double Xrad, double R[9])
{
  double s = sin(Xrad);
  double c = cos(Xrad);

  R[0] = 1.0;    R[1] = 0.0;    R[2] = 0.0; 
  R[3] = 0.0;    R[4] = c;      R[5] = -s;
  R[6] = 0.0;    R[7] = s;      R[8] = c;

  return;
}

void
UFV::convYradToRMat(const double Yrad, double R[9])
{
  double s = sin(Yrad);
  double c = cos(Yrad);

  R[0] = c;      R[1] = 0.0;    R[2] = s; 
  R[3] = 0.0;    R[4] = 1.0;    R[5] = 0.0;
  R[6] = -s;     R[7] = 0.0;    R[8] = c;

  return;
}

void
UFV::convZradToRMat(const double Zrad, double R[9])
{
  double s = sin(Zrad);
  double c = cos(Zrad);

  R[0] = c;      R[1] = -s;     R[2] = 0.0; 
  R[3] = s;      R[4] = c;      R[5] = 0.0;
  R[6] = 0.0;    R[7] = 0.0;    R[8] = 1.0;

  return;
}

void
UFV::convXYZradToRMat(const double Xrad, const double Yrad, const double Zrad,
                      double R[9])
{
  double sx = sin(Xrad);  double cx = cos(Xrad);
  double sy = sin(Yrad);  double cy = cos(Yrad);
  double sz = sin(Zrad);  double cz = cos(Zrad);

  R[0] = cy*cz;   R[1] = -cx*sz + sx*sy*cz;   R[2] = sx*sz + cx*sy*cz; 
  R[3] = cy*sz;   R[4] = cx*cz + sx*sy*sz;    R[5] = -sx*cz + cx*sy*sz;
  R[6] = -sy;     R[7] = sx*cy;               R[8] = cx*cy;
  
  return;
}

void
UFV::convXradToHMat(const double Xrad, double H[12])
{
  double s = sin(Xrad);
  double c = cos(Xrad);

  H[0] = 1.0;    H[1] = 0.0;    H[2] = 0.0; 
  H[4] = 0.0;    H[5] = c;      H[6] = -s;
  H[8] = 0.0;    H[9] = s;      H[10] = c;

  return;
}

void
UFV::convYradToHMat(const double Yrad, double H[12])
{
  double s = sin(Yrad);
  double c = cos(Yrad);

  H[0] = c;      H[1] = 0.0;    H[2] = s; 
  H[4] = 0.0;    H[5] = 1.0;    H[6] = 0.0;
  H[8] = -s;     H[9] = 0.0;    H[10] = c;

  return;
}

void
UFV::convZradToHMat(const double Zrad, double H[12])
{
  double s = sin(Zrad);
  double c = cos(Zrad);

  H[0] = c;      H[1] = -s;     H[2] = 0.0; 
  H[4] = s;      H[5] = c;      H[6] = 0.0;
  H[8] = 0.0;    H[9] = 0.0;    H[10] = 1.0;

  return;
}

void
UFV::convXYZradToHMat(const double Xrad, const double Yrad, const double Zrad,
                 double H[9])
{
  double sx = sin(Xrad);  double cx = cos(Xrad);
  double sy = sin(Yrad);  double cy = cos(Yrad);
  double sz = sin(Zrad);  double cz = cos(Zrad);

  H[0] = cy*cz;   H[1] = -cx*sz + sx*sy*cz;   H[2] = sx*sz + cx*sy*cz; 
  H[4] = cy*sz;   H[5] = cx*cz + sx*sy*sz;    H[6] = -sx*cz + cx*sy*sz;
  H[8] = -sy;     H[9] = sx*cy;               H[10] = cx*cy;
  
  return;
}

void
UFV::mult3x3Matrix(const double A[9], const double B[9], double C[9])
{
  double buf[3];
  for(int i=0; i<3; i++)
  {
    for(int k=0; k<3; k++)
    {
      buf[k] = A[i*3+k];
    }
    for(int j=0; j<3; j++)
    {
      double s=0.0;
      for(int k=0; k<3; k++)
      {
        s += buf[k] * B[k*3+j];
      }
      C[i*3+j] = s;
    }
  }
  return;
}

void
UFV::mult4x4Matrix(const double A[16], const double B[16], double C[16])
{
  double buf[4];
  for(int i=0; i<4; i++)
  {
    for(int k=0; k<4; k++)
    {
      buf[k] = A[i*4+k];
    }
    for(int j=0; j<4; j++)
    {
      double s=0.0;
      for(int k=0; k<4; k++)
      {
        s += buf[k] * B[k*4+j];
      }
      C[i*4+j] = s;
    }
  }
  return;
}

static UFV::Point2D
get_projection(const UFV::Point3D &p, const float *P, const float *Q)
{
  int i;
  float m[3];

  for(i=0; i<3; i++)
    m[i] = P[i*3]*p.x + P[i*3+1]*p.y + P[i*3+2]*p.z + Q[i];

  if(m[2] != 0.0)
  {
    m[0] /= m[2];
    m[1] /= m[2];
  }

  UFV::Point2D poi(m[0],m[1]);

  return(poi);
}

UFV::Point2D
UFV::getProjection(const double A[9],
                   const double R[9],
                   const double T[3],
                   const UFV::Point3D &poi)
{
  float P[9],Q[3];

  for(int i=0; i<3; i++)
  {
    for(int j=0; j<3; j++)
    {
      P[i*3+j] = A[i*3]*R[j] + A[i*3+1]*R[j+3] + A[i*3+2]*R[j+6];
    }
    Q[i] = A[i*3]*T[0] + A[i*3+1]*T[1] + A[i*3+2]*T[2];
  }

  return get_projection(poi, P, Q);
}
