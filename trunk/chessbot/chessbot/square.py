#!/usr/bin/env python

import roslib
roslib.load_manifest('chessbot')
import rospy


import actionlib

import cart_interp.msg
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *

def execute_pose(pose):
    global arm_client
    posestamped = PoseStamped()
    posestamped.header.stamp = rospy.Time.now()
    posestamped.header.frame_id = 'torso_lift_link'
    posestamped.pose = pose
    goal = cart_interp.msg.CartesianArmServerGoal(setpoint = posestamped)
    arm_client.send_goal(goal)
    arm_client.wait_for_result()

def main():
    global arm_client
    rospy.init_node('chessbot')
    arm_client = actionlib.SimpleActionClient('r_cart_action_server', cart_interp.msg.CartesianArmServerAction)
    arm_client.wait_for_server()
    out_of_sight = Pose()
    out_of_sight.position = Point(0.842470855969, -0.185501577783, 0.315879531841)
    out_of_sight.orientation = Quaternion(-0.00403457514937, -0.580539478508, 0.00205700227215, 0.814219506545)
    while (not rospy.is_shutdown()):
        print('a')
        out_of_sight.position = Point(0.642470855969, -0.185501577783, 0.315879531841)
        execute_pose(out_of_sight)
        print('b')
        out_of_sight.position =Point(0.702470855969, -0.185501577783, 0.315879531841)
        execute_pose(out_of_sight)
        print('c')
        out_of_sight.position =Point(0.702470855969, -0.125501577783, 0.315879531841)
        execute_pose(out_of_sight)
        print('d')
        out_of_sight.position =Point(0.642470855969, -0.125501577783, 0.315879531841)
        execute_pose(out_of_sight)

if __name__ == '__main__':
    main()
