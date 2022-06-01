#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@file transmittance_monitor.py
@brief

@author Yasushi SUMI <y.sumi@aist.go.jp>

Copyright (C) 2021 AIST
Released under the MIT license
https://opensource.org/licenses/mit-license.php
"""

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
import math

from emulated_srs.msg import Transmittance
from emulated_srs.msg import ExpSetup

class TransmittanceMonitor(object):
    THRESHOLD_ATTENUATION = math.log(0.02)

    def __init__(self):
        #rospy.init_node('transmittance_monitor', anonymous=True)
        rospy.init_node('transmittance_monitor')

        self._sensor_name = ""
        self._dist_testpiece = 1000.0
        self._sub_exp = rospy.Subscriber('experimental_setup', ExpSetup, self._callback_exp)

        _sub_img = message_filters.Subscriber('/processing_unit/measurer/depth_labeled/image_raw', Image)
        _sub_tns = message_filters.Subscriber('/transmittance', Transmittance)

        _queue_size = 100
        _delay = 0.1

        #self._sync = message_filters.ApproximateTimeSynchronizer([_sub_img, _sub_tns], _queue_size, _delay, reset=True)
        self._sync = message_filters.ApproximateTimeSynchronizer([_sub_img, _sub_tns], _queue_size, _delay)
        self._sync.registerCallback(self._callback)

        self._prev_stamp = rospy.Time(0)
        
        self._bridge = CvBridge()

        rospy.loginfo("Initialization: OK")

        return

    def _callback_exp(self, msg):
        self._sensor_name = msg.name_sensor
        self._dist_testpiece = msg.dist_testpiece
        rospy.loginfo("ExpSetup: %s %.1f", self._sensor_name, self._dist_testpiece)
        self._sub_exp.unregister()
        return

    def _callback(self, msg_img, msg_tns):
        try:
            self._depth_map = self._bridge.imgmsg_to_cv2(msg_img, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self._trans_value = msg_tns.trans
        self._trans_distance = msg_tns.distance
        self._trans_wavelength = msg_tns.wavelength

        rospy.logwarn_once("An image has been subscribed: %d x %d x %d" % \
                self._depth_map.shape)
        
        itime = msg_img.header.stamp.to_sec()
        ttime = msg_tns.header.stamp.to_sec()
        rospy.loginfo("Time: img %.2f, tns %.2f, dif %.2f", itime, ttime, itime-ttime)
        
        if self._prev_stamp < msg_img.header.stamp:
            # to avoid display problems when looping data
            self.display()
        
        self._prev_stamp = msg_img.header.stamp
        
        return

    def calc_MOR(self, x, t):
        mor = -1.0
        try:
            mor = 0.001 * x * self.THRESHOLD_ATTENUATION / math.log(t)
            rospy.loginfo("MOR: %f, %f, %f, %f", mor, x, t, self.THRESHOLD_ATTENUATION)
        except ValueError as e:
            rospy.logerr("MOR ValueError %f", t)
        return mor

    def calc_transmittance_at(self, x, t, dist):
        t_at = -1.0
        try:
            alpha = -math.log(t)/x
            t_at = math.exp(-alpha*dist)
            rospy.loginfo("Tat: %f, %f, %f, %f", t_at, x, t, dist)
        except ValueError as e:
            rospy.logerr("T_at ValueError: %f", t)
        except ZeroDivisionError as e:
            rospy.logerr("ZeroDivision")

        return t_at

    def display(self):
        rospy.loginfo("Transmittance: %f", self._trans_value)
        
        _height, _width, _ = self._depth_map.shape
        _trans_at = self.calc_transmittance_at(self._trans_distance, 
                                                self._trans_value,
                                                self._dist_testpiece)
        _mor = self.calc_MOR(self._trans_distance, self._trans_value)
        _text_trans_org = "T_{:.0f}: {:.3f}".format(self._trans_distance, self._trans_value)
        _text_trans_at = "T_{:.0f}: {:.3f}".format(self._dist_testpiece, _trans_at)
        _text_MOR = "MOR: {:.1f}".format(_mor)
        _fontface = cv2.FONT_HERSHEY_SIMPLEX
        _fontscale = 1.0
        _thickness = 2
        _margin = 16
        (_, _h), _ = cv2.getTextSize(_text_trans_org, _fontface, _fontscale, _thickness)

        cv2.putText(self._depth_map,
            text=_text_trans_org,
            org=(_margin,_h+_margin), 
            fontFace=_fontface,
            fontScale=_fontscale,
            color=(0,255,255),
            thickness=_thickness,
            lineType=cv2.LINE_AA)
        cv2.putText(self._depth_map,
            text=_text_trans_at,
            org=(_margin,_h*2+_margin*2), 
            fontFace=_fontface,
            fontScale=_fontscale,
            color=(0,255,255),
            thickness=_thickness,
            lineType=cv2.LINE_AA)
        cv2.putText(self._depth_map,
            text=_text_MOR,
            org=(_margin,_h*3+_margin*3), 
            fontFace=_fontface,
            fontScale=_fontscale,
            color=(0,255,255),
            thickness=_thickness,
            lineType=cv2.LINE_AA)    
        cv2.imshow('transmittance_monitor', self._depth_map)

        k = cv2.waitKey(50) & 0xFF
        if k == 27:
            rospy.logwarn("Escaped")

        elif k == ord('c'):
            rospy.logwarn("Canceled")

if __name__ == '__main__':
    try:
        #cv2.namedWindow(winname='my_drawing')
        TransmittanceMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

