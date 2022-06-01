#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
@file transmissometer.py
@brief

@author Masato Kodama <kodama.masato@aist.go.jp>

Copyright (C) 2021 AIST
Released under the MIT license
https://opensource.org/licenses/mit-license.php
"""

import rospy
from std_msgs.msg import Float64
from emulated_srs.msg import Transmittance 
from datetime import datetime
from socket import socket, AF_INET, SOCK_DGRAM

def publishTransmittance():
    # for ros
    transmsg = Transmittance()
    pub = rospy.Publisher('transmittance', Transmittance, queue_size=10)
    rospy.init_node('Transmittance', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    
    # for udp
    host = ''
    port = 5678 # 適当なポート番号です。特に設計して決めた数値ではないので、別のシステムとかぶる場合はWin側と合わせ変更してください
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind((host, port))
    
    rospy.loginfo('受信待ちです,通信強制停止はctrl+z')
    while not rospy.is_shutdown():
    	# PC(Windows)側からは”epoch(ms),制御対象波長(###.###),透過率(###.###)”のデータがASCII文字列で飛んできます。
    	#ex)1653631013768,781.711,95.951
    	#透過率はPC側で計算で出しているため、光源の揺れ具合によって100.000を超える場合があります。
        
        data, address = s.recvfrom(8192) # socket.recvfrom(bufsize[, flags])　ソケットからデータを受信しタプル(bytes, address)で返す
        
        tmpstr = data.decode(encoding='ASCII') # b'1653631013768,781.711,95.951' -> '1653631013768,781.711,95.951'
        arstr = tmpstr.split(',')              # '1653631013768,781.711,95.951' -> ['1653631013768','781.711','95.951']
       
        #publish data input
        transmsg.header.seq += 1
        transmsg.header.stamp.secs = int(arstr[0]) // 1000
        transmsg.header.stamp.nsecs = (int(arstr[0]) % 1000) * (10 ** 6) # nsec単位でデータを入れるため msを10の6乗倍
        transmsg.wavelength = float(arstr[1])
        transmsg.trans = float(arstr[2]) * 0.01
        transmsg.distance = float(arstr[3])

        pub.publish(transmsg)
#        rospy.loginfo(data)
        rospy.loginfo(transmsg)
#       rate.sleep()
        
    s.close()
    
if __name__ == '__main__':
    try:
        publishTransmittance()
    except rospy.ROSInterruptException:
        pass
