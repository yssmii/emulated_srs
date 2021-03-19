/**
 * @file Keydefs.h
 * @brief
 *
 * @author Yasushi SUMI (AIST)
 *
 * Copyright (C) 2008  AIST
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 */

#ifndef __LIBESRS_UFV_KEYDEFS_H__
#define __LIBESRS_UFV_KEYDEFS_H__

#if defined(HAVE_X11_KEYSYM_H)
#include <X11/keysym.h>
#else
#define XK_Return 0xff0d
#define XK_Escape 0xff1b
#define XK_space 0x0020
#define XK_c 0x0063
#define XK_h 0x0068
#define XK_j 0x006a
#define XK_k 0x006b
#define XK_l 0x006c
#define XK_n 0x006e
#define XK_q 0x0071
#define XK_r 0x0072
#define XK_s 0x0073
#define XK_x 0x0078
#define XK_z 0x007a
#define XK_H 0x0048
#define XK_J 0x004a
#define XK_K 0x004b
#define XK_L 0x004c
#endif


#endif
