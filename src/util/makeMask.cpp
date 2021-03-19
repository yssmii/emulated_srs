/**
 * @file  makeMask.cpp
 * @brief mask image generator
 *
 * @author Yasushi SUMI <y.sumi@aist.go.jp>
 * 
 * Copyright (C) 2013  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ctype.h>
#include <assert.h>
#include <vector>
#include <iostream>

#ifdef _WIN32
#include "opt/getopt.h"
#endif
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 

typedef struct
{
  int x;
  int y;
  int width;
  int height;
  int iwidth;
  int iheight;
  std::string ofname;
} Options;

/*********************************************************************
 * Print usage and exit
 */
static void
usage_exit(void)
{
  std::cerr << "  Usage: testMask [options] x y width height" << std::endl;
  
  std::cerr << "   x y:       left sholder point" << std::endl;
  std::cerr << "   width:     width" << std::endl;
  std::cerr << "   height:    height" << std::endl;
  std::cerr << "   -g wxh:    width and height of baseimage (640x480 default)"
            << std::endl;
  std::cerr << "   -n fname:  mask image filename (\"MASK.png\" default)"
            << std::endl;
  std::cerr << "   -h:        print this" << std::endl;
  exit(1);
}

static int
get_command_options(int ac, char *av[], Options *opt)
{
  int c;
  extern char *optarg;
  extern int optind;
  int ret;

  while(1)
  {
    c = getopt(ac,av,"hg:n:");
    if(c == -1) break;

    switch(c)
    {
      case 'h':
        usage_exit();
        break;
      case 'g':
        ret = sscanf(optarg, "%dx%d", &(opt->iwidth), &(opt->iheight));
        if(ret != 2)
          usage_exit();
        break;
      case 'n':
        opt->ofname = optarg;
        break;
      default:
        usage_exit();
        break;
    }
  }

  if(ac - optind != 4)
    usage_exit();

  opt->x = atoi(av[optind]);
  opt->y = atoi(av[optind+1]);
  opt->width = atoi(av[optind+2]);
  opt->height = atoi(av[optind+3]);

#ifdef DEBUG
  std::cout << opt->ofname << " ("
            << opt->x << ", "
            << opt->y << ": "
            << opt->width << ", "
            << opt->height << ") :"
            << opt->iwidth << ", "
            << opt->iheight << std::endl;
#endif

  if(opt->width > opt->iwidth || opt->height > opt->iheight)
    usage_exit();

  return(optind);
}

int
main(int argc, char *argv[]) 
{
  Options opt = {0, 0, 0, 0, 640, 480, "MASK.png"};
  get_command_options(argc, argv, &opt);

  cv::Mat img = cv::Mat::zeros(cv::Size(opt.iwidth, opt.iheight), CV_8U);

  cv::Point p1(opt.x, opt.y);
  cv::Point p2(p1.x + opt.width - 1, p1.y + opt.height - 1);

  cv::rectangle(img, p1, p2, 255, cv::FILLED);

  cv::imwrite(opt.ofname, img);

  return(0);
}
