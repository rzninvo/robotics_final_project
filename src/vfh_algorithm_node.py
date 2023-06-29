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
        self.current_heading = 0

        self.fig = plt.figure()
        self.fig.set_figheight(self.fig.get_figheight() * 2)
        self.axes = [self.fig.add_subplot(211), self.fig.add_subplot(212)]
        self.hists = []
        self.axvlines = []
        self.axhlines = []
        self.axes[0].set_title('Polar Obstacle Density Histogram')
        self.axes[1].set_title('Candidate Valleys')

        self.vfh_service = rospy.Service('vfh_planner_service', vfh_planner, self.vfh_planner_callback)
        
    def laser_callback(self, msg: LaserScan):
        self.laser_data = msg.ranges

    def calculate_active_window(self):
        maximum_distance = self.a / self.b
        # We know that: maximum_distance = sqrt(2)*(window_size)/2
        window_size = (math.sqrt(2) * maximum_distance)
        active_window = {}
        counter = 0
        for laser_reading in self.laser_data:
            reading_x = abs(laser_reading * math.cos(math.radians(counter)))
            reading_y = abs(laser_reading * math.sin(math.radians(counter)))
            maximum_width = (window_size) / 2
            if max(maximum_width, reading_x, reading_y) == maximum_width:
                confidence = 1
                if (maximum_width != 0):
                    confidence = laser_reading / (2 * maximum_width) 
                    confidence = abs(confidence - 1)
                active_window[counter] = (confidence ** 2) * (self.a - (self.b * laser_reading))
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
        length = len(self.POD_histogram)
        for k in range(len(self.POD_histogram)):
            sum_element = self.POD_histogram[k] * self.l
            for i in range(self.l):
                sum_element += (self.POD_histogram[k - (i + 1)] + self.POD_histogram[(k + (i + 1)) % length]) * (self.l - i)
            smoothed_element = sum_element / ((2 * self.l) + 1)
            smoothed_POD_histogram.append(smoothed_element)
            if (smoothed_element < self.valley_threshold):
                candidate_valleys.append(smoothed_element)
                candidate_valleys_index.append(k)
            else:
                candidate_valleys.append(0)
        return smoothed_POD_histogram, candidate_valleys, candidate_valleys_index
    
    def calculate_goal_angle(self):
        goal_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        alpha_rotation = goal_angle - self.current_heading
        beta_rotation = alpha_rotation + 2 * math.pi
        gamma_rotation = alpha_rotation - 2 * math.pi
        
        rotations = [abs(alpha_rotation), abs(beta_rotation), abs(gamma_rotation)]
        min_indx = rotations.index(min(rotations))

        rotation = 0
        if min_indx == 0:
            rotation = alpha_rotation
        elif min_indx == 1:
            rotation = beta_rotation
        elif min_indx == 2:
            rotation = gamma_rotation

        if (rotation < 0):
            rotation += 2 * math.pi
        return rotation
    
    def calculate_angle(self):
        if len(self.candidate_valleys_index) == 0:
            return 0
        goal_angle = np.rad2deg(self.calculate_goal_angle())
        self.goal_angle = goal_angle
        goal_sector = round(goal_angle / self.alpha)
        #rospy.loginfo(f'GOAL ANGLE = {goal_angle} |-| GOAL SECTOR = {goal_sector} |-| CURRENT_HEADING = {np.rad2deg(self.current_heading)}')
        minimum = 1000
        k_n = -1
        for i in range(len(self.candidate_valleys_index)):
            x = self.candidate_valleys_index[i]
            min_distance = min(abs(x - goal_sector), abs(abs(x - goal_sector) - len(self.smooth_POD_histogram)))
            if min_distance < minimum:
                if ((x * self.alpha) < 90) or ((x * self.alpha) > 270):
                    minimum = min_distance
                    k_n = x
        if k_n == -1:
            k_n_index = np.argmin([min(abs(x - goal_sector), abs(abs(x - goal_sector) - len(self.smooth_POD_histogram))) for x in self.candidate_valleys_index])
            k_n = self.candidate_valleys_index[k_n_index]
        k_f = k_n
        if (k_n == goal_sector):
            return math.radians(goal_sector * self.alpha)
        elif k_n > goal_sector:
            historgram_length = len(self.smooth_POD_histogram)
            for i in range(1, self.s_max + 1):
                if (self.smooth_POD_histogram[(k_n + i) % historgram_length] < self.valley_threshold) and ((((k_n + i) % historgram_length) * self.alpha) <= math.radians(90)):
                    k_f = k_n + i
                else:
                    break
            return math.radians(round(((k_n + k_f) / 2.0) % historgram_length) * self.alpha)
        else:
            historgram_length = len(self.smooth_POD_histogram)
            for i in range(1, self.s_max + 1):
                if (self.smooth_POD_histogram[(k_n - i)] < self.valley_threshold) and (((k_n - i) % historgram_length) * self.alpha >= math.radians(270)):
                    k_f = k_n - i
                else:
                    break
            return math.radians(round(((k_n + k_f) / 2.0) % historgram_length) * self.alpha)

    def vfh_planner_callback(self, req : vfh_planner) -> vfh_plannerResponse:
        self.alpha = req.alpha
        self.a = req.a
        self.b = req.b
        self.l = req.l
        self.current_x = req.current_x
        self.current_y = req.current_y
        self.current_heading = req.current_heading
        self.goal_x = req.goal_x
        self.goal_y = req.goal_y
        self.valley_threshold = req.valley_threshold
        self.s_max = req.s_max
        if (len(self.laser_data) != 0):
            self.active_window = self.calculate_active_window()
            self.smooth_POD_histogram, self.candidate_valleys, self.candidate_valleys_index = self.calculate_smoothed_POD_histogram()
            goal_angle = self.calculate_angle()
            maximum = max(self.smooth_POD_histogram)
            minimum = min(self.smooth_POD_histogram)
            #     divnorm = TwoSlopeNorm(vmin= self.valley_threshold, vcenter= minimum, vmax= maximum)
            # elif (self.valley_threshold > maximum):
            #         divnorm = TwoSlopeNorm(vmin= minimum, vcenter= maximum, vmax= self.valley_threshold)
            # else:
            if (self.valley_threshold >= minimum) and (self.valley_threshold <= maximum):
                self.divnorm = TwoSlopeNorm(vmin= minimum, vcenter= self.valley_threshold, vmax= maximum)
            self.cmap = plt.get_cmap('RdYlGn_r')(self.divnorm(self.smooth_POD_histogram)) 
            response = vfh_plannerResponse()
            response.direction = goal_angle
            response.h_c = max(max(self.smooth_POD_histogram[0:7]), max(self.smooth_POD_histogram[65:72]))
            return response
        else:
            response = vfh_plannerResponse()
            response.direction = math.radians(0)
            response.h_c = 0
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
                self.axhlines.append(self.axes[i].axhline(y=self.valley_threshold, color='r', linestyle='--', linewidth=1, label = 'Valley Threshold'))
                self.axvlines.append(self.axes[i].axvline(x=self.goal_angle, color='b', linestyle='-', linewidth=1, label = 'Target_Angle'))
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
            self.axvlines[0].set_xdata(self.goal_angle)
            self.axvlines[1].set_xdata(self.goal_angle)
            self.axhlines[0].set_ydata(self.valley_threshold)
            self.axhlines[1].set_ydata(self.valley_threshold)

    def plot_histogram(self):
        animation = FuncAnimation(self.fig, self.animate_plot, cache_frame_data=False)
        plt.show(block= True)

if __name__ == "__main__":
    controller = VFHAlgorithm()
    controller.plot_histogram()
    # r = rospy.Rate(500)
    # r.sleep()
    rospy.spin()