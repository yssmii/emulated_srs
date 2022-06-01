#!/usr/bin/env python
# license removed for brevity
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import String

from emulated_srs.msg import Transmittance

class DummyTransmissometer(object):
    def __init__(self, r):
        rospy.init_node('dummy_transmissometer')
        rospy.Subscriber('/processing_unit/measurer/depth/image_raw',
            Image, self._callback)
        self._pub = rospy.Publisher('transmittance', Transmittance, queue_size=10)
        self._rate = rospy.Rate(r)
        self._trans = Transmittance()
        self._trans.wavelength = 780.0
        self._trans.distance = 6000.0
        self._trans.trans = 0.0001
        self._stamp = rospy.Time.now()

    def _callback(self,msg):
        self._stamp = msg.header.stamp
        rospy.loginfo("Time: %.2f", self._stamp.to_sec())
        rospy.loginfo("Image: %d x %d ", msg.width, msg.height)
 

    def _publish(self):
        self._trans.trans += 0.1
        if self._trans.trans > 1.0:
            self._trans.trans = 0.0001
        self._trans.header.stamp = self._stamp
        self._pub.publish(self._trans)
        self._rate.sleep()

if __name__ == '__main__':
    try:
        trans = DummyTransmissometer(20)  # 4hz
        
        while not rospy.is_shutdown():
            trans._publish()

    except rospy.ROSInterruptException:
        pass
