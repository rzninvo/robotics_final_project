#!/usr/bin/python3

import rospy

from sensor_msgs.msg import LaserScan
from robotics_final_project.srv import vfh_planner, vfh_plannerResponse
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import TwoSlopeNorm

class VFHAlgorithm:
    
    def __init__(self) -> None:
        
        rospy.init_node("vfh_algorithm_node" , anonymous=True)
        self.laser_data = []
        self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)

        self.alpha = 0
        self.a = 0
        self.b = 0
        self.valley_threshold = 0
        self.s_max = 0
        self.l = 0
        self.current_x = 0
        self.current_y = 0
        self.goal_x = 0
        self.goal_y = 0
        self.active_window = None
        self.draw_flag = False
        self.smooth_POD_histogram = []
        self.POD_histogram = []
        self.candidate_valleys = []
        self.candidate_valleys_index = []
        self.goal_angle = 0

        self.fig = plt.figure()
        self.fig.set_figheight(self.fig.get_figheight() * 2)
        self.axes = [self.fig.add_subplot(211), self.fig.add_subplot(212)]
        self.hists = []
        self.axvlines = []
        self.axes[0].set_title('Polar Obstacle Density Histogram')
        self.axes[1].set_title('Candidate Valleys')

        self.vfh_service = rospy.Service('vfh_planner_service', vfh_planner, self.vfh_planner_callback)
        
    def laser_callback(self, msg: LaserScan):
        self.laser_data = msg.ranges

    def calculate_active_window(self):
        maximum_distance = self.a / self.b
        # We know that: maximum_distance = sqrt(2)*(window_size - 1)/2
        window_size = (math.sqrt(2) * maximum_distance) + 1
        active_window = {}
        counter = 90
        for laser_reading in self.laser_data:
            reading_x = abs(laser_reading * math.cos(math.radians(counter)))
            reading_y = abs(laser_reading * math.sin(math.radians(counter)))
            maximum_width = (window_size - 1) / 2
            if max(maximum_width, reading_x, reading_y) == maximum_width:
                active_window[counter] = self.a - (self.b * laser_reading)
            else:
                active_window[counter] = 0
            counter += 1
            counter = counter % 360
        return active_window
    
    def calculate_smoothed_POD_histogram(self):
        active_window_list = [self.active_window[i] for i in range(0, 360)]
        self.POD_histogram = [sum(active_window_list[self.alpha * k : self.alpha * (k + 1)]) for k in range(int(360 / self.alpha))]
        smoothed_POD_histogram = []
        candidate_valleys = []
        candidate_valleys_index = []
        for k in range(len(self.POD_histogram)):
            sum_element = self.POD_histogram[k] * self.l
            length = len(self.POD_histogram)
            for i in range(self.l):
                sum_element += (self.POD_histogram[k - (i + 1)] + self.POD_histogram[(k + (i + 1)) % length]) * (self.l - i)
            smoothed_element = sum_element / ((2 * self.l) + 1)
            smoothed_POD_histogram.append(smoothed_element)
            if smoothed_element < self.valley_threshold:
                candidate_valleys.append(smoothed_element)
                candidate_valleys_index.append(k)
            else:
                candidate_valleys.append(0)
        return smoothed_POD_histogram, candidate_valleys, candidate_valleys_index
    
    def calculate_angle(self):
        goal_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        if (goal_angle < 0):
            goal_angle += 2 * math.pi
        goal_angle = np.rad2deg(goal_angle)
        goal_sector = round(goal_angle / self.alpha)
        k_n = np.argmin([abs(x - goal_sector) for x in self.candidate_valleys_index])
        k_f = k_n
        if (self.candidate_valleys_index[k_n] == goal_sector):
            return math.radians(goal_sector * self.alpha)
        elif (self.candidate_valleys_index[k_n] > goal_sector):
            candidate_length = len(self.candidate_valleys_index)
            historgram_length = len(self.smooth_POD_histogram)
            for i in range(1, self.s_max + 1):
                if abs(self.candidate_valleys_index[(k_n + i) % candidate_length] - self.candidate_valleys_index[k_f]) == 1:
                    k_f = (k_n + i) % candidate_length
                elif abs(self.candidate_valleys_index[(k_n + i) % candidate_length] - self.candidate_valleys_index[k_f]) == (historgram_length - 1):
                    k_f = (k_n + i) % candidate_length
                else:
                    break
            return math.radians((((self.candidate_valleys_index[k_n] + self.candidate_valleys_index[k_f]) % historgram_length) / 2.0) * self.alpha)
        else:
            candidate_length = len(self.candidate_valleys_index)
            historgram_length = len(self.smooth_POD_histogram)
            for i in range(1, self.s_max + 1):
                if abs(self.candidate_valleys_index[(k_n - i)] - self.candidate_valleys_index[k_f]) == 1:
                    k_f = (k_n - i)
                elif abs(self.candidate_valleys_index[(k_n - i)] - self.candidate_valleys_index[k_f]) == (historgram_length - 1):
                    k_f = (k_n - i)
                else:
                    break
            return math.radians((((self.candidate_valleys_index[k_n] + self.candidate_valleys_index[k_f]) % historgram_length) / 2.0) * self.alpha)

    def vfh_planner_callback(self, req : vfh_planner) -> vfh_plannerResponse:
        self.alpha = req.alpha
        self.a = req.a
        self.b = req.b
        self.l = req.l
        self.current_x = req.current_x
        self.current_y = req.current_y
        self.goal_x = req.goal_x
        self.goal_y = req.goal_y
        self.valley_threshold = req.valley_threshold
        self.s_max = req.s_max
        if (len(self.laser_data) != 0):
            self.active_window = self.calculate_active_window()
            self.smooth_POD_histogram, self.candidate_valleys, self.candidate_valleys_index = self.calculate_smoothed_POD_histogram()
            self.goal_angle = self.calculate_angle()
            maximum = max(self.smooth_POD_histogram)
            minimum = min(self.smooth_POD_histogram)
            divnorm = TwoSlopeNorm(vmin= minimum, vcenter= self.valley_threshold, vmax= maximum)
            self.cmap = plt.get_cmap('RdYlGn_r')(divnorm(self.smooth_POD_histogram))    
            response = vfh_plannerResponse()
            response.direction = self.goal_angle
            return response
        else:
            response = vfh_plannerResponse()
            response.direction = math.radians(90)
            return response
    
    def animate_plot(self, i):
        if (self.alpha != 0) and (self.draw_flag == False) and (len(self.smooth_POD_histogram) != 0):
            bins = [x * 5 for x in range(1, int(360 / self.alpha) + 1)]
            for i in range(len(self.axes)):
                self.axes[i].cla()
                if i == 0:
                    self.hists.append(self.axes[i].bar(bins, height=self.smooth_POD_histogram, width= -self.alpha, align='edge', edgecolor='white', color = self.cmap))
                else:
                    self.hists.append(self.axes[i].bar(bins, height=self.candidate_valleys, width= -self.alpha, align='edge', edgecolor='white', color = self.cmap))
                self.axes[i].set_xticks([0, 90, 180, 270, 360])
                self.axes[i].set_xticklabels(['0°', '90°', '180°', '270°', '360°'])
                y_ticks = list(self.axes[0].get_yticks())
                y_ticks.append(self.valley_threshold)
                y_ticks.sort()
                self.axes[i].set_yticks(y_ticks)
                self.axes[i].set_xlabel('Angle')
                self.axes[i].set_ylabel('POD')
                self.axes[i].axhline(y=self.valley_threshold, color='r', linestyle='--', linewidth=1, label = 'Valley Threshold')
                goal_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
                if (goal_angle < 0):
                    goal_angle += 2 * math.pi
                goal_angle = np.rad2deg(goal_angle)
                self.axvlines.append(self.axes[i].axvline(x=goal_angle, color='b', linestyle='-', linewidth=1, label = 'Target_Angle'))
                self.axes[i].legend()
            self.axes[0].set_title('Polar Obstacle Density Histogram')
            self.axes[1].set_title('Candidate Valleys')
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            self.draw_flag = True
        elif self.draw_flag == True:
            hist1 = self.hists[0]
            hist2 = self.hists[1]
            for i, b in enumerate(hist1):
                b.set_height(self.smooth_POD_histogram[i])
                b.set_facecolor(self.cmap[i])
            for i, b in enumerate(hist2):
                b.set_height(self.candidate_valleys[i])
                b.set_facecolor(self.cmap[i])
            goal_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
            if (goal_angle < 0):
                goal_angle += 2 * math.pi
            goal_angle = np.rad2deg(goal_angle)
            self.axvlines[0].set_xdata(goal_angle)
            self.axvlines[1].set_xdata(goal_angle)

    def plot_histogram(self):
        animation = FuncAnimation(self.fig, self.animate_plot, cache_frame_data=False, interval = 1)
        plt.show(block= True)

if __name__ == "__main__":
    controller = VFHAlgorithm()
    controller.plot_histogram()
    rospy.spin()