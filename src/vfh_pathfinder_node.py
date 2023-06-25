#!/usr/bin/python3

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
    
    def __init__(self) -> None:
        
        rospy.init_node("vfh_pathfinder_node" , anonymous=False)

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # getting specified parameters
        self.linear_spead = rospy.get_param("/vfh_pathfinder_node/linear_spead")
        self.angular_k_p = rospy.get_param("/vfh_pathfinder_node/angular_k_p")
        self.angular_k_i = rospy.get_param("/vfh_pathfinder_node/angular_k_i")
        self.angular_k_d = rospy.get_param("/vfh_pathfinder_node/angular_k_d")
        self.dt = rospy.get_param("/vfh_pathfinder_node/dt")
        self.goal_x = rospy.get_param("/vfh_pathfinder_node/goal_x")
        self.goal_y = rospy.get_param("/vfh_pathfinder_node/goal_y")
        self.epsilon = rospy.get_param("/vfh_pathfinder_node/epsilon")
        self.alpha = rospy.get_param("/vfh_pathfinder_node/alpha")
        self.coefficient_a = rospy.get_param("/vfh_pathfinder_node/coefficient_a")
        self.coefficient_b = rospy.get_param("/vfh_pathfinder_node/coefficient_b")
        self.coefficient_l = rospy.get_param("/vfh_pathfinder_node/coefficient_l")
        self.valley_threshold = rospy.get_param("/vfh_pathfinder_node/valley_threshold")
        self.s_max = rospy.get_param("/vfh_pathfinder_node/s_max")

        self.rate = 1/self.dt

        self.r = rospy.Rate(self.rate)

        # defining the states of our robot
        self.GO, self.FOLLOW, self.ROTATE = 0, 1, 2
        self.state = self.GO
    
    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw

    def get_bb_from_server(self):
        rospy.wait_for_service('vfh_planner_service')
        try:
            gbb_data_service = rospy.ServiceProxy('vfh_planner_service', vfh_planner)
            response : vfh_plannerResponse = gbb_data_service(self.alpha, self.coefficient_a, self.coefficient_b, 
                                    self.coefficient_l, self.valley_threshold, self.s_max, self.goal_x, self.goal_y)
            
        except rospy.ServiceException as e:
           rospy.loginfo(f'ERROR from vfh_planner_service: {e}')

    def run(self):

        errors_avg = 0
        msg = rospy.wait_for_message("/odom" , Odometry)
        gamma = 0
        angular_sum_i_theta = 0
        angular_prev_theta_error = 0
        
        while (not rospy.is_shutdown()):

            if self.state == self.GO:
                self.get_bb_from_server()
                twist = Twist()
                twist.linear.x = self.linear_spead
                self.cmd_vel.publish(twist)
                continue

        twist = Twist()  
        self.cmd_vel.publish(Twist())

if __name__ == "__main__":
    controller = VFHPathFinder()
    controller.run()