#!/usr/bin/env python
import time

import roslib
roslib.load_manifest('cart_interp')
import rospy
import math
import actionlib

import cart_interp.msg
from geometry_msgs.msg import *

targetpose = None
currentpose = None

class CartesianArmServer:
    def __init__(self, name, controller):
        global targetpose, currentpose, linear_position_error
        targetpose = None
        currentpose = None
        linear_position_error = None
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, cart_interp.msg.CartesianArmServerAction, execute_cb = self.execute)
        rospy.Subscriber("/%s/state/x" % controller, PoseStamped, self.callback)
        self.pub_posestamped = rospy.Publisher("/%s/command_pose" % controller, PoseStamped)
    
    def callback(self, data):
        global linear_position_error, currentpose, targetpose
        currentpose = data.pose
        if (targetpose is not None):
            linear_position_error=math.sqrt(math.pow(targetpose.position.x-data.pose.position.x,2)+math.pow(targetpose.position.y-data.pose.position.y,2)+math.pow(targetpose.position.z-data.pose.position.z,2))
    
    def execute(self, goal):
        global targetpose, currentpose, linear_position_error
        success = True
        rate = rospy.Rate(1.0)
        targetpose = goal.setpoint.pose
        linear_position_error = 9999
        while (linear_position_error > 0.02 or linear_position_error is None):
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False
                break
            self.pub_posestamped.publish(goal.setpoint)
            feedback = cart_interp.msg.CartesianArmServerFeedback()
            feedback.currentpoint = currentpose
            rate.sleep()
        if success:
            result = cart_interp.msg.CartesianArmServerResult()
            result.endpoint = currentpose
            self._as.set_succeeded(result)
        
def main():
    rospy.init_node('cartesian_arm_server')
    server = CartesianArmServer(rospy.get_name(), rospy.myargv()[1])
    rospy.spin()
    
if __name__=='__main__':
    main()
