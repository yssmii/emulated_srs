#!/usr/bin/env python
# license removed for brevity
from cmath import e
from tkinter import E
import rospy
from std_msgs.msg import String
from emulated_srs.msg import ExpSetup

def broadcaster():
    rospy.init_node('experimental_setup_broadcaster', anonymous=True, disable_signals=True)

    pub = rospy.Publisher('experimental_setup', ExpSetup, queue_size=1, latch=True)
    sname = "UNKNOWN"
    tdist = -1.0

    try:
        #sname = rospy.get_param('/processing_unit/measurer/sensor_name')
        sname = rospy.get_param('~sensor_name')
        tdist = rospy.get_param('~dist_testpiece')
    except KeyError:
        rospy.logerr("value not set")
        rospy.signal_shutdown("value not set")
    
    if(tdist<0.0):
        rospy.logerr("invalid distance %f" % tdist)
        rospy.signal_shutdown("invalid distance %f" % tdist)

    rospy.loginfo("Broadcast ExtSetup: %s %.1f", sname, tdist)
    esetup = ExpSetup()
    esetup.name_sensor = sname
    esetup.dist_testpiece = tdist

    esetup.header.stamp = rospy.Time.now()
   
    pub.publish(esetup)

    rospy.spin()

    return

if __name__ == '__main__':
    try:
        broadcaster()
    except rospy.ROSInterruptException:
        pass
