#!/usr/bin/env python

import roslib
roslib.load_manifest('cart_interp')
import rospy

import actionlib

import cart_interp.msg
from geometry_msgs.msg import *

def cartesian_move_right_arm_client():
    client = actionlib.SimpleActionClient('cartesian_move_right_arm', cart_interp.msg.CartesianMoveRightArmAction)
    client.wait_for_server()
    
    posestamped = PoseStamped()
    posestamped.header.stamp = rospy.Time.now()
    posestamped.header.frame_id = 'torso_lift_link'
    posestamped.pose.orientation.x=-0.00403457514937
    posestamped.pose.orientation.y=-0.580539478508
    posestamped.pose.orientation.z=0.00205700227215
    posestamped.pose.orientation.w=0.814219506545
    
    while not rospy.is_shutdown():
        posestamped.pose.position=Point(0.806302276543, -0.385346655457, 0.310545735804)
        goal = cart_interp.msg.CartesianMoveRightArmGoal(setpoint = posestamped)
        client.send_goal(goal)
        client.wait_for_result()
        print('move 1')
        posestamped.pose.position=Point(0.806302276543, -0.185346655457, 0.310545735804)
        goal = cart_interp.msg.CartesianMoveRightArmGoal(setpoint = posestamped)
        client.send_goal(goal)
        client.wait_for_result()
        print('move 2')
        posestamped.pose.position=Point(0.646302276543, -0.185346655457, 0.310545735804)
        goal = cart_interp.msg.CartesianMoveRightArmGoal(setpoint = posestamped)
        client.send_goal(goal)
        client.wait_for_result()
        print('move 3')
        posestamped.pose.position=Point(0.646302276543, -0.385346655457, 0.310545735804)
        goal = cart_interp.msg.CartesianMoveRightArmGoal(setpoint = posestamped)
        client.send_goal(goal)
        client.wait_for_result()
        print('move 4')


if __name__ == '__main__':
    try:
        rospy.init_node('cartesian_move_right_arm_client')
        cartesian_move_right_arm_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
