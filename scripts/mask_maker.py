#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@file mask_maker.py
@brief

@author Yasushi SUMI <y.sumi@aist.go.jp>

Copyright (C) 2021 AIST
Released under the MIT license
https://opensource.org/licenses/mit-license.php
"""
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from emulated_srs.srv import SetMask
#from emulated_srs.msg import Obstacle

class MaskMaker(object):
    def __init__(self):
        rospy.init_node('mask_maker')
        rospy.Subscriber('/processing_unit/measurer/depth/image_raw',
            Image, self._callback)
        
        rospy.wait_for_service('/processing_unit/measurer/set_mask')
        self._service_mask_set = rospy.ServiceProxy('/processing_unit/measurer/set_mask', SetMask)

        self._flg_draw_rectangle = False
        self._flg_button_pressed = False
        self._flg_event_listener = False
        self._ix, self._iy = -1,-1
        self._cx, self._cy = -1,-1

        self._bridge = CvBridge()
        self._min_img = np.zeros((1,1),np.uint8)

    def _callback(self, msg):
        try:
            self._depth_map = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        if __debug__:
            rospy.logwarn_once("An image has been subscribed: %d x %d x %d" % \
                self._depth_map.shape)
        
        self.display()

        return

    def display(self):
        #cv2.namedWindow(winname='my_drawing')    
        #cv2.imshow('my_drawing', self.depth_map)
        if self._flg_draw_rectangle == True:
            cv2.rectangle(self._depth_map,(self._ix,self._iy),(self._cx,self._cy),(0,255,0),1)        
        cv2.imshow('mask_maker', self._depth_map)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            rospy.logwarn("Escaped")

        elif k == ord('c'):
            self._flg_draw_rectangle = False
            self._ix, self._iy = -1,-1
            self._cx, self._cy = -1,-1
            rospy.logwarn("Canceled")

        elif k == ord('s'):
            height, width, _ = self._depth_map.shape
            mask_image = np.zeros((height, width, 1), np.uint8)
            cv2.rectangle(mask_image, (self._ix, self._iy), (self._cx, self._cy), 255, -1)

            if __debug__:
                cv2.imshow('mask', mask_image)
                cv2.waitKey(1)

            imsg = self._bridge.cv2_to_imgmsg(mask_image, encoding="passthrough")
            res = self._service_mask_set("start", imsg)
            rospy.logwarn("Sent and saved: %s" % res)

        elif k == ord('p'):
            imsg = self._bridge.cv2_to_imgmsg(self._min_img, encoding="passthrough")
            res = self._service_mask_set("pause", imsg)
            rospy.logwarn("Paused: %s" % res)

        elif k == ord('r'):
            imsg = self._bridge.cv2_to_imgmsg(self._min_img, encoding="passthrough")
            res = self._service_mask_set("restart", imsg)
            rospy.logwarn("Restarted: %s" % res)

        if self._flg_event_listener == False:
            cv2.setMouseCallback('mask_maker',self.set_rectangle)
            self._flg_event_listener = True
            rospy.logwarn("Set EventListener")

        return

    def set_rectangle(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._flg_draw_rectangle = True
            self._flg_button_pressed = True
            self._ix, self._iy = x,y
            self._cx, self._cy = x,y

        elif event == cv2.EVENT_MOUSEMOVE:
            if self._flg_button_pressed == True:
                self._cx, self._cy = x,y
                #self.map_copied = self.depth_map.copy()
                #cv2.imshow('my_drawing', img_cp)
                #cv2.rectangle(self.map_copied,(self.ix,self.iy),(x,y),(0,255,0),-1)
                #cv2.imshow('my_drawing',self.map_copied)

        elif event == cv2.EVENT_LBUTTONUP:
            self._flg_button_pressed = False
            #cv2.imshow('my_drawing', img_cp)
            # cv2.rectangle(img,(ix,iy),(x,y),(0,255,0),-1)

if __name__ == '__main__':
    try:
        #cv2.namedWindow(winname='my_drawing')
        MaskMaker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

