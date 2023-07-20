#!/usr/bin/python3
"""
    This node is responsible for the robot's movement. It uses the VFH algorithm to calculate the best direction to move to.
    It also uses a PID controller to control the robot's angular velocity. The robot moves to the next position when it reaches 
    the epsilon distance from the goal position. The goal position is the next position in the goal_coordinates list.

    This script requires the following parameters to be set in the launch file:
        - linear_spead: the linear speed of the robot
        - angular_k_p: the proportional gain of the PID controller
        - angular_k_i: the integral gain of the PID controller
        - angular_k_d: the derivative gain of the PID controller
        - dt: the time interval between each iteration of the loop
        - epsilon: the distance from the goal position that the robot should reach before moving to the next position
        - alpha: the angle between the robot's heading and the goal position
        - coefficient_a: the coefficient of the first term in the potential function
        - coefficient_b: the coefficient of the second term in the potential function
        - coefficient_l: the coefficient of the third term in the potential function
        - valley_threshold: the threshold of the valley
        - s_max: the maximum distance from the robot to the obstacle
        - h_m: the maximum height of the valley
    This script requires the following topics to be published:
        - /cmd_vel: the topic that the robot subscribes to in order to move
        - /odom: the topic that the robot subscribes to in order to get its position
    This script provides the following services:
        - vfh_planner_service: the service that the robot uses to get the goal angle and the height of the valley
    This script uses the following libraries:
        - rospy: the ROS Python library
        - tf: the ROS library that is used to convert quaternions to euler angles
        - nav_msgs.msg: the ROS library that is used to get the robot's position
        - geometry_msgs.msg: the ROS library that is used to move the robot
        - matplotlib.pyplot: the Python library that is used to plot the robot's path
        - sensor_msgs.msg: the ROS library that is used to get the laser scan data
        - numpy: the Python library that is used to calculate the robot's position
        - robotics_final_project.srv: the ROS library that is used to provide the vfh_planner_service
        - math: the Python library that is used to calculate the robot's position
    The author of this script is Roham Zendehdel Nobari. All rights reserved.
    Author email: rzninvo@gmail.com
    ROS version: Noetic
    Python version: 3.8.10
    Date: 15 June 2021
    LICENSE: MIT LICENSE, https://opensource.org/licenses/MIT
"""

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import numpy as np
from robotics_final_project.srv import vfh_planner, vfh_plannerResponse

import math

class VFHPathFinder:
    """
        This class is responsible for the robot's movement. It uses the VFH algorithm to calculate the best direction to move to.
        It also uses a PID controller to control the robot's angular velocity. The robot moves to the next position when it reaches
        the epsilon distance from the goal position. The goal position is the next position in the goal_coordinates list.

        Attributes
        ----------
        cmd_vel : Publisher
            the topic that the robot subscribes to in order to move
        linear_spead : float
            the linear speed of the robot
        angular_k_p : float
            the proportional gain of the PID controller
        angular_k_i : float
            the integral gain of the PID controller
        angular_k_d : float
            the derivative gain of the PID controller
        dt : float
            the time interval between each iteration of the loop
        epsilon : float
            the distance from the goal position that the robot should reach before moving to the next position
        alpha : float
            the angle between the robot's heading and the goal position
        coefficient_a : float
            the coefficient of the first term in the potential function
        coefficient_b : float
            the coefficient of the second term in the potential function
        coefficient_l : float
            the coefficient of the third term in the potential function
        valley_threshold : float
            the threshold of the valley
        s_max : float
            the maximum distance from the robot to the obstacle
        h_m : float
            the maximum height of the valley
        rate : Rate
            the rate of the loop
        next_position : int
            the index of the next position in the goal_coordinates list
        goal_coordinates : list
            the list of the goal positions
        thresholds : list
            the list of the thresholds of the valleys
    """
    
    # initializing the node
    def __init__(self) -> None:
        
        rospy.init_node("vfh_pathfinder_node" , anonymous=False)

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # getting specified parameters
        self.linear_spead = rospy.get_param("/vfh_pathfinder_node/linear_spead")
        self.angular_k_p = rospy.get_param("/vfh_pathfinder_node/angular_k_p")
        self.angular_k_i = rospy.get_param("/vfh_pathfinder_node/angular_k_i")
        self.angular_k_d = rospy.get_param("/vfh_pathfinder_node/angular_k_d")
        self.dt = rospy.get_param("/vfh_pathfinder_node/dt")
        self.next_position = 0
        self.goal_coordinates = [(4.4, 0), (3.66, 4.47), (2.19, 1.46), (0.55, 1.83), (1.0, 5.43), (2.05, 5.64), (2.89, 6.60), (3.90, 6.83), (4.55, 5.65), (5.54, 5.59), (5.8, 3.08), (7.30, 3.2), (7.21, 6.64), (13, 6.64)]
        self.thresholds = [2, 3, 4, 4, 3, 4, 3.4, 3.5, 4, 3, 3, 3, 4, 3]
        self.epsilon = rospy.get_param("/vfh_pathfinder_node/epsilon")
        self.alpha = rospy.get_param("/vfh_pathfinder_node/alpha")
        self.coefficient_a = rospy.get_param("/vfh_pathfinder_node/coefficient_a")
        self.coefficient_b = rospy.get_param("/vfh_pathfinder_node/coefficient_b")
        self.coefficient_l = rospy.get_param("/vfh_pathfinder_node/coefficient_l")
        self.valley_threshold = rospy.get_param("/vfh_pathfinder_node/valley_threshold")
        self.s_max = rospy.get_param("/vfh_pathfinder_node/s_max")
        self.h_m = rospy.get_param("/vfh_pathfinder_node/h_m")

        self.rate = 1/self.dt

        #self.r = rospy.Rate(1000)
    
    # getting the robot's heading from the /odom topic
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw

    # getting the robot's distance from the goal position
    def get_distance_from_goal(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        pos = msg.pose.pose.position
        return math.sqrt(((self.goal_coordinates[self.next_position][0] - pos.x) ** 2) + ((self.goal_coordinates[self.next_position][1] - pos.y) ** 2))

    # getting the goal angle and the height of the valley from the vfh_planner_service. The goal angle is the angle between the robot's heading and the goal position.
    def get_goal_angle_from_server(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        heading = self.get_heading()
        rospy.wait_for_service('vfh_planner_service')
        try:
            pos = msg.pose.pose.position
            gbb_data_service = rospy.ServiceProxy('vfh_planner_service', vfh_planner)
            response : vfh_plannerResponse = gbb_data_service(self.alpha, self.coefficient_a, self.coefficient_b, 
                                    self.coefficient_l, self.valley_threshold, self.s_max, pos.x, pos.y, heading,
                                      self.goal_coordinates[self.next_position][0], self.goal_coordinates[self.next_position][1])
            return response.direction, response.h_c
        except rospy.ServiceException as e:
           rospy.loginfo(f'ERROR from vfh_planner_service: {e}')

    # calculating the rotation error using the goal angle and the robot's heading. 
    def calculate_rotation_error(self, rotation_goal):
        heading = self.get_heading()
        if (rotation_goal <= math.radians(180)):
            if (rotation_goal <= math.radians(90)):
                alpha_rotation = rotation_goal
            else:
                alpha_rotation = 0#rotation_goal * 0.002
        else:
            if (rotation_goal >= math.radians(270)):
                alpha_rotation = rotation_goal
            else:
                alpha_rotation = 0#rotation_goal * 0.002
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

        #rospy.loginfo(f'ROTATION_GOAL: {np.rad2deg(rotation_goal)}, HEADING: {np.rad2deg(heading)}, ROTATION: {np.rad2deg(rotation)}')

        return rotation

    def run(self):

        errors_avg = 0
        msg = rospy.wait_for_message("/odom" , Odometry)
        gamma = 0
        angular_sum_i_theta = 0
        angular_prev_theta_error = 0
        
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_spead
        move_cmd.angular.z = 0

        while (not rospy.is_shutdown()):

            self.valley_threshold = self.thresholds[self.next_position]

            self.cmd_vel.publish(move_cmd)
            rotation_goal, h_c = self.get_goal_angle_from_server()
            err_gamma = self.calculate_rotation_error(rotation_goal)
            # if err_gamma == 0:
            #     err_gamma == angular_prev_theta_error

            # PID controller for angular velocity of the robot
            angular_P = self.angular_k_p * err_gamma
            angular_I = self.angular_k_i * angular_sum_i_theta
            angular_D = self.angular_k_d * (err_gamma - angular_prev_theta_error)

            # h_c for controlling the linear velocity of the robot
            h_c_prime = min(self.h_m, h_c)
            v = self.linear_spead * (1 - (h_c_prime / self.h_m))

            move_cmd.angular.z = angular_P + angular_I + angular_D
            move_cmd.linear.x = v
            angular_prev_theta_error = err_gamma

            if self.get_distance_from_goal() <= self.epsilon:
                self.next_position += 1
            rospy.loginfo(f'NEXT_POSITION = ({self.goal_coordinates[self.next_position][0]}, {self.goal_coordinates[self.next_position][1]}')
            #rospy.loginfo(f'NODE_PATHFINDER: {rospy.get_time()}')
            #self.r.sleep()

        twist = Twist()  
        self.cmd_vel.publish(Twist())

if __name__ == "__main__":
    controller = VFHPathFinder()
    controller.run()