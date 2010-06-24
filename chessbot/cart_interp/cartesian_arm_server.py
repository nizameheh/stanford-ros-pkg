#!/usr/bin/env python
import time

import roslib
roslib.load_manifest('cart_interp')
import rospy
import math
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
    
    def callback(self, data):
        self.currentpose = data.pose
        if (self.targetpose is not None):
            self.linear_position_error=euclidean_distance(self.targetpose.position, data.pose.position)
            rospy.logdebug(str(self.linear_position_error))
    
    def execute(self, goal):
        success = True
        rate = rospy.Rate(10.0)
        self.targetpose = goal.setpoint.pose
        self.linear_position_error = None
        while (self.linear_position_error is None or self.linear_position_error > self.success_threshold) and not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False
                break
            setpoint = goal.setpoint
            goal_distance = euclidean_distance(currentpose, targetpose)
            if goal_distance > max_lead:
                scaling_factor = max_lead/goal_distance
                setpoint.point.x = (setpoint.point.x - currentpose.position.x)*scaling_factor+currentpose.position.x
                setpoint.point.y = (setpoint.point.y - currentpose.position.y)*scaling_factor+currentpose.position.y
                setpoint.point.z = (setpoint.point.z - currentpose.position.z)*scaling_factor+currentpose.position.z
                
            self.pub_posestamped.publish(setpoint)
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
