#!/usr/bin/env python
import time

import roslib
roslib.load_manifest('cart_interp')
import rospy
import math
import copy
import actionlib

import cart_interp.msg
from geometry_msgs.msg import *

class CartesianArmServer:
    def __init__(self, name):
        self.success_threshold = rospy.get_param('success_threshold')
        self.max_lead = rospy.get_param('max_lead')
        self.targetpose = None
        self.currentpose = None
        self.linear_position_error = None
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, cart_interp.msg.CartesianArmServerAction, execute_cb=self.execute)
        rospy.Subscriber("current_pose", PoseStamped, self.callback)
        self.pub_posestamped = rospy.Publisher("command_pose", PoseStamped)
        self.pub_targetpose = rospy.Publisher("target_pose", PoseStamped)
    
    def callback(self, data):
        self.currentpose = data.pose
        if (self.targetpose is not None):
            self.linear_position_error = euclidean_distance(self.targetpose.position, self.currentpose.position)
            rospy.loginfo('Position error: ' + str(self.linear_position_error))
    
    def execute(self, goal):
        success = True
        rate = rospy.Rate(10.0)
        self.targetpose = goal.setpoint.pose
        self.pub_targetpose.publish(goal.setpoint)
        self.linear_position_error = None
        while (self.linear_position_error is None):
            rate.sleep()
        while self.linear_position_error > self.success_threshold and not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False
                break
            waypoint = PoseStamped()
            waypoint.pose = copy.deepcopy(self.targetpose)
            waypoint.header.stamp = rospy.Time.now()
            waypoint.header.frame_id =goal.setpoint.header.frame_id
            if self.linear_position_error > self.max_lead:
                #if the endpoint is too far away, lead the arm there gradually
                scaling_factor = self.max_lead/self.linear_position_error
                waypoint.pose.position.x = (self.targetpose.position.x - self.currentpose.position.x)*scaling_factor+self.currentpose.position.x
                waypoint.pose.position.y = (self.targetpose.position.y - self.currentpose.position.y)*scaling_factor+self.currentpose.position.y
                waypoint.pose.position.z = (self.targetpose.position.z - self.currentpose.position.z)*scaling_factor+self.currentpose.position.z
            self.pub_posestamped.publish(waypoint)
            feedback = cart_interp.msg.CartesianArmServerFeedback()
            feedback.currentpoint = self.currentpose
            rate.sleep()
        if success:
            result = cart_interp.msg.CartesianArmServerResult()
            result.endpoint = self.currentpose
            self._as.set_succeeded(result)

def euclidean_distance(point1, point2):
    return math.sqrt((point1.x-point2.x) ** 2 + (point1.y-point2.y) ** 2 + (point1.z-point2.z) ** 2)

def main():
    rospy.init_node('cartesian_arm_server')
    server = CartesianArmServer(rospy.get_name())
    rospy.spin()
    
if __name__=='__main__':
    main()
