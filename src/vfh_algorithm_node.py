#!/usr/bin/python3
"""
This module implements the VFH algorithm. It subscribes to the /scan topic and publishes the direction to the /direction topic.
It also provides a service that can be called to get the direction and the h_c value.
There is also the plot_histogram function which plots the histogram and the candidate valleys.

This script requires the following topics to be published:
    * /scan: the topic that the robot subscribes to in order to get the laser scan data
This script provides the following services:
    * vfh_planner_service: the service that the robot uses to get the goal angle and the height of the valley
This script requires the following libraries:
    * rospy: the ROS Python library
    * sensor_msgs.msg: the ROS library that is used to get the laser scan data
    * robotics_final_project.srv: the project package
    * math: the Python library that is used to calculate the robot's position
    * numpy: the Python library that is used to calculate the robot's position
    * matplotlib.pyplot: the Python library that is used to plot the robot's path
    * matplotlib.animation: the Python library that is used to animate the plot
    * matplotlib.colors: the Python library that is used to calculate the color map
    * matplotlib.figure: the Python library that is used to create the figure
    * matplotlib.axes: the Python library that is used to create the axes
    * matplotlib.lines: the Python library that is used to create the lines
    * matplotlib.patches: the Python library that is used to create the patches
    * matplotlib.collections: the Python library that is used to create the collections
    * matplotlib.transforms: the Python library that is used to create the transforms
    * matplotlib.ticker: the Python library that is used to create the tickers
The author of this script is Roham Zendehdel Nobari. All rights reserved.
Author Email: rzninvo@gmail.com
ROS version: Noetic
Python version: 3.8.10
Date: 14 June 2021
LICENSE: MIT LICENSE, https://opensource.org/licenses/MIT
"""

import rospy

from sensor_msgs.msg import LaserScan
from robotics_final_project.srv import vfh_planner, vfh_plannerResponse
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import TwoSlopeNorm

class VFHAlgorithm:
    """
    This class implements the VFH algorithm. It subscribes to the /scan topic and publishes the direction to the /direction topic.
    It also provides a service that can be called to get the direction and the h_c value. 
    There is also the plot_histogram function which plots the histogram and the candidate valleys.

    Attributes
    ----------
    laser_data : list
        The list of laser readings
    laser_subscriber : rospy.Subscriber
        The subscriber to the /scan topic
    alpha : float
        The angle between two consecutive bins of the histogram
    a : float
        The weight of the distance from the robot to the obstacle
    b : float
        The weight of the distance from the robot to the obstacle
    valley_threshold : float
        The threshold value for the valley
    s_max : int
        The maximum number of bins to be considered for the valley
    l : int
        The number of bins to be considered for the smoothing
    current_x : float
        The x coordinate of the robot
    current_y : float
        The y coordinate of the robot
    goal_x : float
        The x coordinate of the goal
    goal_y : float
        The y coordinate of the goal
    active_window : dict
        The dictionary containing the active window
    draw_flag : bool
        The flag that indicates whether the histogram has been drawn or not
    smooth_POD_histogram : list
        The list containing the smoothed histogram
    POD_histogram : list
        The list containing the histogram
    candidate_valleys : list
        The list containing the candidate valleys
    candidate_valleys_index : list
        The list containing the indices of the candidate valleys
    goal_angle : float
        The angle between the robot and the goal
    current_heading : float
        The current heading of the robot
    fig : matplotlib.figure.Figure
        The figure that contains the histogram and the candidate valleys
    axes : list
        The list containing the axes of the figure
    hists : list
        The list containing the histograms
    axvlines : list
        The list containing the vertical lines
    axhlines : list
        The list containing the horizontal lines
    vfh_service : rospy.Service
        The service that can be called to get the direction and the h_c value
    """
    
    # The constructor of the class. It initializes the node and the subscriber.
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

    # This function is called when a new laser scan is received
    def laser_callback(self, msg: LaserScan):
        self.laser_data = msg.ranges

    # This function calculates the active window. It returns a dictionary with the key being the angle and the value being the POD
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
            # if (reading_x > maximum_width) or (reading_y < maximum_width)
            if max(maximum_width, reading_x, reading_y) == maximum_width:
                confidence = 1
                if (maximum_width != 0):
                    # Calculating the confidece based on how near the object is to the robot. The farthest in the window is 0.5 and the closest is 1.
                    confidence = laser_reading / (2 * maximum_width) 
                    confidence = abs(confidence - 1)
                active_window[counter] = (confidence ** 2) * (self.a - (self.b * laser_reading))
            else:
                active_window[counter] = 0
            counter += 1
            counter = counter % 360
        return active_window
    
    # This function calculates the smoothed POD histogram and the candidate valleys
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
            # Adding the sector to the candidate valleys if it's smaller than the valley threshold
            if (smoothed_element < self.valley_threshold):
                candidate_valleys.append(smoothed_element)
                candidate_valleys_index.append(k)
            else:
                candidate_valleys.append(0)
        return smoothed_POD_histogram, candidate_valleys, candidate_valleys_index
    
    # This function calculates the goal angle. It calculates the best angle to reach the goal.
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
    
    # This function calculates the angle that the robot should rotate to avoid the obstacle.
    def calculate_angle(self):
        if len(self.candidate_valleys_index) == 0:
            return 0
        goal_angle = np.rad2deg(self.calculate_goal_angle())
        self.goal_angle = goal_angle
        goal_sector = round(goal_angle / self.alpha)
        # Additional code for debugging
        # rospy.loginfo(f'GOAL ANGLE = {goal_angle} |-| GOAL SECTOR = {goal_sector} |-| CURRENT_HEADING = {np.rad2deg(self.current_heading)}')
        minimum = 1000
        k_n = -1
        for i in range(len(self.candidate_valleys_index)):
            x = self.candidate_valleys_index[i]
            # Calculating the minimum distance between the goal sector and the candidate valley
            min_distance = min(abs(x - goal_sector), abs(abs(x - goal_sector) - len(self.smooth_POD_histogram)))
            if min_distance < minimum:
                # Filtering the valleys that are in the opposite direction of the goal
                if ((x * self.alpha) < 90) or ((x * self.alpha) > 270):
                    minimum = min_distance
                    k_n = x
        # If there is no valley left after the filtering, we choose the valley with the minimum distance even if it's in the opposite direction of the goal.
        if k_n == -1:
            k_n_index = np.argmin([min(abs(x - goal_sector), abs(abs(x - goal_sector) - len(self.smooth_POD_histogram))) for x in self.candidate_valleys_index])
            k_n = self.candidate_valleys_index[k_n_index]
        k_f = k_n
        # If the nearest valley is in the same direction as the goal, we return the angle of the valley
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

    # This function is called when the service is called. It returns the direction and the h_c value.
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
                # Calculating the color map based on the valley threshold
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
    
    # This function is called to animate the plot. It is called every time the plot needs to be updated.
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
            # Updating the histogram data
            for i, b in enumerate(hist1):
                b.set_height(self.smooth_POD_histogram[i])
                b.set_facecolor(self.cmap[i])
            for i, b in enumerate(hist2):
                b.set_height(self.candidate_valleys[i])
                b.set_facecolor(self.cmap[i])
            # Updating the vertical and horizontal lines
            self.axvlines[0].set_xdata(self.goal_angle)
            self.axvlines[1].set_xdata(self.goal_angle)
            self.axhlines[0].set_ydata(self.valley_threshold)
            self.axhlines[1].set_ydata(self.valley_threshold)

    # This function plots the histogram and the candidate valleys.
    def plot_histogram(self):
        animation = FuncAnimation(self.fig, self.animate_plot, cache_frame_data=False)
        plt.show(block= True)

if __name__ == "__main__":
    controller = VFHAlgorithm()
    controller.plot_histogram()
    # r = rospy.Rate(500)
    # r.sleep()
    rospy.spin()