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
        self.targetpose = None
        self.currentpose = None
        self.linear_position_error = None
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, cart_interp.msg.CartesianArmServerAction, execute_cb=self.execute)
        rospy.Subscriber("/cartesian_arm_controller/state/x", PoseStamped, self.callback)
        self.pub_posestamped = rospy.Publisher("/cartesian_arm_controller/command_pose", PoseStamped)
    
    def callback(self, data):
        self.currentpose = data.pose
        if (self.targetpose is not None):
            self.linear_position_error=math.sqrt((self.targetpose.position.x-data.pose.position.x ** 2) + (self.targetpose.position.y-data.pose.position.y ** 2) + (self.targetpose.position.z-data.pose.position.z ** 2))
            rospy.logdebug(str(self.linear_position_error))
    
    def execute(self, goal):
        success = True
        rate = rospy.Rate(1.0)
        self.targetpose = goal.setpoint.pose
        self.linear_position_error = None
        while (self.linear_position_error is None or self.linear_position_error > self.success_threshold) and not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False
                break
            self.pub_posestamped.publish(goal.setpoint)
            feedback = cart_interp.msg.CartesianArmServerFeedback()
            feedback.currentpoint = self.currentpose
            rate.sleep()
        if success:
            result = cart_interp.msg.CartesianArmServerResult()
            result.endpoint = self.currentpose
            self._as.set_succeeded(result)
        
def main():
    rospy.init_node('cartesian_arm_server')
    server = CartesianArmServer(rospy.get_name())
    rospy.spin()
    
if __name__=='__main__':
    main()
