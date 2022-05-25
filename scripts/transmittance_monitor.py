#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np

from emulated_srs.msg import Transmittance
from emulated_srs.msg import ExpSetup

class TransmittanceMonitor(object):
    def __init__(self):
        rospy.init_node('transmittance_monitor', anonymous=True)

        #self._sensor_name = ""
        #self._dist_testpiece = 0.0
        #self._sub_exp = rospy.Subscriber('experimental_setup', ExpSetup, self._callback_exp)

        _sub_img = message_filters.Subscriber('/processing_unit/measurer/depth_labeled/image_raw', Image)
        _sub_tns = message_filters.Subscriber('transmittance', Transmittance)

        _queue_size = 100
        _delay = 0.5

        self._sync = message_filters.ApproximateTimeSynchronizer([_sub_img, _sub_tns], _queue_size, _delay, reset=True)
        self._sync.registerCallback(self._callback)

        self._prev_stamp = rospy.Time(0)
        
        self._bridge = CvBridge()

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

    def display(self):
        rospy.loginfo("Transmittance: %f", self._trans_value)
        
        _height, _width, _ = self._depth_map.shape
        _text = "{:.2f}".format(self._trans_value*100.0)
        _fontface = cv2.FONT_HERSHEY_SIMPLEX
        _fontscale = 0.6
        _thickness = 1
        _margin = 4
        (_w, _h), _baseline = cv2.getTextSize(_text, _fontface, _fontscale, _thickness)

        cv2.putText(self._depth_map,
            text=_text,
            org=(_width-_w-_margin,_h+_margin), 
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

