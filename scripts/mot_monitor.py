#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@file mot_monitor.py
@brief

@author Yasushi SUMI <y.sumi@aist.go.jp>

Copyright (C) 2022 AIST
Released under the MIT license
https://opensource.org/licenses/mit-license.php
"""

import rospy

import numpy as np
import math
import matplotlib.pyplot as plt
import time

from emulated_srs.msg import Transmittance
from emulated_srs.msg import ExpSetup
from emulated_srs.msg import ObstacleGroup

class MOTGraph():
    """
    ref. https://www.wizard-notes.com/entry/python/matplotlib-realtime-plot-line
    """
    def __init__(
        self,
        x_tick,
        length,
        xlabel="Time",
        title="MOT",
        label=None,
        color="c",
        marker='.',
        alpha=1.0,
        linewidth=1.0,
        ylim=None,
        reset_after_loop=False
    ):
        self.x_tick = x_tick 
        self.length = length
        self.color = color
        self.marker = marker
        self.linewidth = linewidth
        self.alpha = 1.0
        self.ylim = ylim
        self.label = label
        self.xlabel = xlabel
        self.title = title
        self.line = None
        self.reset_after_loop = reset_after_loop

        # プロット初期化
        self._init_plot()
    
    def _init_plot(self):
        # x[]の初期値。0未満でx_tick刻みのnp.array
        #self.x_vec = np.arange(0, self.length) * self.x_tick \
        #             - self.length * self.x_tick
        self.x_vec = np.zeros(self.length)
        self.y_vec_t = np.zeros(self.length)
        self.y_vec_d = np.full(self.length, -1.0)

        plt.ion()

        fig = plt.figure(figsize=(10,6))
        self.ax_t = fig.add_subplot(111)
        self.ax_d = self.ax_t.twinx()
        self.ax_t.grid(True)
        self.ax_d.grid(False)

        self.line_t = self.ax_t.plot(self.x_vec, self.y_vec_t, 
                            self.marker, color=self.color, 
                            ls="-",
                            linewidth=self.linewidth,
                            alpha=self.alpha)        
        self.line_d = self.ax_d.plot(self.x_vec, self.y_vec_d, 
                            self.marker, color="m", 
                            ls="-",
                            linewidth=self.linewidth,
                            alpha=self.alpha)      

        if self.ylim is not None:
            self.ax_t.set_ylim([self.ylim[0], self.ylim[1]])
        self.ax_d.set_ylim([-300.0, 6300.0])
        self.ax_t.set_xlabel("Time [sec]")
        self.ax_t.set_ylabel("Transmittance")
        self.ax_d.set_ylabel("Obstacle distance [mm]")
        plt.title(self.title)
        #plt.grid()
        plt.show()
        
        self.index = 0
        #self.x_data = -self.x_tick
        self.pretime = 0.0
        self.fps = 0.0
        self._x_min = 0.0

    def set_dist_testpiece(self, dist):
        self.ax_t.set_ylabel(f"Transmittance at {dist}")

    def reset_data(self, xval, yval_t, yval_d):
        self.x_vec[:] = xval
        self.y_vec_t[:] = yval_t
        self.y_vec_d[:] = yval_d
        self._x_min = min(0, xval)
    
    def update_index(self):
        self.index = self.index + 1 if self.index < self.length-1 else 0
        
    def update_ylim(self, y_data):
        ylim = self.line_t[0].axes.get_ylim()
        if   y_data < ylim[0]:
            self.ax_t.set_ylim([y_data*1.1, ylim[1]])
        elif y_data > ylim[1]:
            self.ax_t.set_ylim([ylim[0], y_data*1.1])
            
    def compute_fps(self):
        curtime = time.time()
        time_diff = curtime - self.pretime
        self.fps = 1.0 / (time_diff + 1e-16)
        self.pretime = curtime 
        
    def update(self, x_data, y_data_t, y_data_d):
        # プロットする配列の更新
        #self.x_data += self.x_tick
        self.y_vec_t[self.index] = y_data_t
        self.y_vec_d[self.index] = y_data_d
        
        y_pos = self.index + 1 if self.index < self.length else 0

        tmp_y_vec_t = np.r_[self.y_vec_t[y_pos:self.length], self.y_vec_t[0:y_pos]]
        self.line_t[0].set_ydata(tmp_y_vec_t)

        tmp_y_vec_d = np.r_[self.y_vec_d[y_pos:self.length], self.y_vec_d[0:y_pos]]
        self.line_d[0].set_ydata(tmp_y_vec_d)

        if self.ylim is None:
            self.update_ylim(y_data_t)

        tmp_x_vec = np.roll(self.x_vec, -1)
        tmp_x_vec[self.length - 1] = x_data

        self.x_vec = tmp_x_vec - x_data
        self.line_t[0].set_xdata(self.x_vec)
        self.line_d[0].set_xdata(self.x_vec)
        self._x_min -= x_data

        plt.xlim(self._x_min - self.x_tick, 0)

        plt.title(f"fps: {self.fps:0.1f} Hz")
        #plt.pause(0.01)
        
        # 次のプロット更新のための処理
        self.update_index()
        self.compute_fps()

class MOTMonitor(object):
    THRESHOLD_ATTENUATION = math.log(0.02)

    def __init__(self, fps=4, duration=300):
        #rospy.init_node('transmittance_monitor', anonymous=True)
        rospy.init_node('mot_monitor')

        self._fps = fps
        self._sensor_name = ""
        self._dist_testpiece = 1000.0
        
        self._title = "MOT"
        #self._graph = MOTGraph(1.0/fps, 300*fps)
        self._graph = MOTGraph(1.0/fps, duration*fps, ylim=[-0.05, 1.05])

        self._sub_exp = rospy.Subscriber('experimental_setup', ExpSetup,
                                self._callback_exp)
        
        self._sub_tns = rospy.Subscriber('transmittance', Transmittance,
                                self._callback_trans)

        self._sub_obs = rospy.Subscriber('/processing_unit/measurer/obstacle_group', ObstacleGroup,
                                self._callback_obs)

        self._prev_stamp_trans = rospy.Time(0)
        self._curr_stamp_trans = rospy.Time(0)
        self._prev_stamp_obs = rospy.Time(0)
        self._curr_stamp_obs = rospy.Time(0)

        self._trans_value = 0.0
        self._trans_distance = 0.0
        self._trans_wavelength = 0.0
        self._n_obstacles = 0
        self._obstacle_time = 0

        self._curr_obs_dist = -1.0

        rospy.loginfo("Initialized: fps: %d, duration:%d sec", fps, duration)

        return

    def _callback_exp(self, msg):
        self._sensor_name = msg.name_sensor
        self._dist_testpiece = msg.dist_testpiece
        self._graph.set_dist_testpiece(self._dist_testpiece)
        rospy.loginfo_once("ExpSetup: %s %.1f", self._sensor_name, self._dist_testpiece)
        self._sub_exp.unregister()
        return

    def _callback_trans(self, msg_tns):
        self._trans_value = msg_tns.transmittance
        self._trans_distance = msg_tns.measurement_distance
        self._trans_wavelength = msg_tns.wavelength

        self._curr_stamp_trans = msg_tns.header.stamp
        rospy.loginfo_once("Time Trans: %.2f", self._curr_stamp_trans.to_time())
        return

    def _callback_obs(self, msg_obs):
        self._n_obstacles = msg_obs.n_obstacles
        self._curr_stamp_obs = msg_obs.header.stamp

        if self._n_obstacles > 0:
            self._curr_obs_dist = self.calc_min_obstacle_distance(msg_obs)
            rospy.loginfo_once("Z: %.2f", self._curr_obs_dist)
        rospy.loginfo_once("Obstacles: %d at %.2f",
                        self._n_obstacles, self._curr_stamp_obs.to_time())

        #self.display()

        return

    def calc_min_obstacle_distance(self, msg_obs):
        if msg_obs.n_obstacles <= 0:
            return 0.0
        
        dist = 100000.0
        for obs in msg_obs.data:
            if obs.position_3D.z < dist:
                dist = obs.position_3D.z
        
        return dist * 1000.0

    def calc_MOR(self, x, t):
        mor = -1.0
        try:
            mor = 0.001 * x * self.THRESHOLD_ATTENUATION / math.log(t)
            rospy.loginfo_once("MOR: %f, %f, %f, %f", mor, x, t, self.THRESHOLD_ATTENUATION)
        except ValueError as e:
            rospy.logerr("MOR ValueError %f", t)
        return mor

    def calc_transmittance_at(self, x, t, dist):
        t_at = -1.0
        try:
            alpha = -math.log(t)/x
            t_at = math.exp(-alpha*dist)
            rospy.loginfo_once("Tat: %f, %f, %f, %f", t_at, x, t, dist)
        except ValueError as e:
            rospy.logerr("T_at ValueError: %f", t)
        except ZeroDivisionError as e:
            rospy.logerr("ZeroDivision")

        return t_at

    def display(self):
        while not rospy.is_shutdown():
            if self._prev_stamp_trans != self._curr_stamp_trans:
                self._prev_stamp_trans = self._curr_stamp_trans

            if self._prev_stamp_obs != self._curr_stamp_obs:
                duration_obs = (self._curr_stamp_obs - self._prev_stamp_obs).to_sec()
                _trans_at = self.calc_transmittance_at(self._trans_distance, 
                                                    self._trans_value,
                                                    self._dist_testpiece)
                _mor = self.calc_MOR(self._trans_distance, self._trans_value)

                if duration_obs > 0.0:
                    if self._prev_stamp_obs.is_zero():
                        self._graph.reset_data(0.0, _trans_at, self._curr_obs_dist)
                        self._graph.update(0.0, _trans_at, self._curr_obs_dist)
                    else:
                        self._graph.update(duration_obs, _trans_at, self._curr_obs_dist)
                elif duration_obs < 0.0:
                    self._graph.reset_data(0.0, _trans_at, self._curr_obs_dist)
                    self._graph.update(0.0, _trans_at, self._curr_obs_dist)

                plt.draw()

                self._prev_stamp_obs = self._curr_stamp_obs
            

            plt.pause(0.01)

if __name__ == '__main__':
    try:
        mot = MOTMonitor(fps=4, duration=300)
        mot.display()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass

