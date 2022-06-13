#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy

from sensor_msgs.msg import PointCloud2
from emulated_srs.msg import Transmittance


class DummyTransmissometer(object):
    def __init__(self, r):
        rospy.init_node('dummy_transmissometer')
        rospy.Subscriber('/sensing_unit/camera/depth_registered/points_throttle',
            PointCloud2, self._callback)
        self._pub = rospy.Publisher('transmittance', Transmittance, queue_size=10)
        self._rate = rospy.Rate(r)
        self._trans = Transmittance()
        self._trans.wavelength = 780.0
        self._trans.measurement_distance = 6000.0
        self._trans.transmittance = 0.0001
        self._trans.variance = 0.0
        self._stamp = rospy.Time()
        rospy.loginfo("Time: %.2f", self._stamp.to_sec())
        self._is_started = False

    def _callback(self,msg):
        self._prev_stamp = self._stamp
        self._stamp = msg.header.stamp
        rospy.loginfo("Time: %.2f", self._stamp.to_sec())
        self._is_started = True
 

    def _publish(self):
        if self._is_started:
            if self._prev_stamp < self._stamp:
                self._trans.transmittance += 0.1
                if self._trans.transmittance > 1.0:
                    self._trans.transmittance = 0.0001
                self._trans.header.stamp = self._stamp
                self._pub.publish(self._trans)
        
            self._prev_stamp = self._stamp

        self._rate.sleep()

if __name__ == '__main__':
    try:
        trans = DummyTransmissometer(20)  # 4hz
        
        #rospy.spin()

        while not rospy.is_shutdown():
            trans._publish()

    except rospy.ROSInterruptException:
        pass
