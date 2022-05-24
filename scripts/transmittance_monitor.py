#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np

from emulated_srs.msg import Transmittance

class TransmittanceMonitor(object):
    def __init__(self):
        rospy.init_node('transmittance_monitor', anonymous=True)

        _sub_img = message_filters.Subscriber('/processing_unit/measurer/depth_labeled/image_raw', Image)
        _sub_tns = message_filters.Subscriber('transmittance', Transmittance)

        _queue_size = 100
        _delay = 0.5

        self._sync = message_filters.ApproximateTimeSynchronizer([_sub_img, _sub_tns], _queue_size, _delay, reset=True)
        self._sync.registerCallback(self._callback)

        self._prev_stamp = rospy.Time(0)
        
        self._bridge = CvBridge()


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
            self.display()
        
        self._prev_stamp = msg_img.header.stamp
        
        return

    def display(self):
        rospy.loginfo("Transmittance: %f", self._trans_value)
        
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

