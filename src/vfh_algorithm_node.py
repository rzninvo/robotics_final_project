#!/usr/bin/python3

import rospy

from sensor_msgs.msg import LaserScan
from robotics_final_project.srv import vfh_planner, vfh_plannerResponse
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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
        self.goal_x = 0
        self.goal_y = 0
        self.active_window = None

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.fig.suptitle('Polar Obstacle Density Histogram')

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
        for k in range(len(self.POD_histogram)):
            sum_element = self.POD_histogram[k]
            length = len(self.POD_histogram)
            for i in range(self.l):
                sum_element += (self.POD_histogram[k - (i + 1)] + self.POD_histogram[(k + (i + 1)) % length])
            smoothed_POD_histogram.append(sum_element / ((2 * self.l) + 1))
        return smoothed_POD_histogram

    def vfh_planner_callback(self, req : vfh_planner) -> vfh_plannerResponse:
        self.alpha = req.alpha
        self.a = req.a
        self.b = req.b
        self.l = req.l
        self.goal_x = req.goal_x
        self.goal_y = req.goal_y
        self.valley_threshold = req.valley_threshold
        self.s_max = req.s_max
        self.active_window = self.calculate_active_window()
        self.smooth_POD_histogram = self.calculate_smoothed_POD_histogram()
        response = vfh_plannerResponse()
        return response
    
    def animate_plot(self, i):
        if (self.alpha != 0):
            self.ax.cla()
            bins = [x * 5 for x in range(1, int(360 / self.alpha) + 1)]
            hist = self.ax.bar(bins, height=self.smooth_POD_histogram, width= -self.alpha, align='edge', edgecolor='white')
            self.ax.set_xticks([0, 90, 180, 270, 360])
            self.ax.set_xticklabels(['0°', '90°', '180°', '270°', '360°'])
            self.ax.set_xlabel('Angle')
            self.ax.set_ylabel('POD')
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def plot_histogram(self):
        animation = FuncAnimation(self.fig, self.animate_plot, cache_frame_data=False)
        plt.show(block= True)

if __name__ == "__main__":
    controller = VFHAlgorithm()
    controller.plot_histogram()
    rospy.spin()