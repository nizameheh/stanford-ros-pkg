#!/usr/bin/env python

import roslib
roslib.load_manifest('chessbot')
import rospy
import math

import actionlib

import cart_interp.msg
import chessbot.msg
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *
import tf

class Square:
    def __init__(self, row, col):
        self.row = row
        self.col = col        
class Chessbot:
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, chessbot.msg.ChessbotAction, execute_cb = self.execute)

    def execute(self, goal):
        global latch_chessboard
        latch_chessboard = True
        destination = Square(goal.destination_row, goal.destination_col)
        origin = Square(goal.origin_row, goal.origin_col)
        if goal.capture:
            move_piece(destination, Square(-2, -2))
        move_piece(origin, destination)
        self._as.set_succeeded()
        latch_chessboard = False

def square_to_coordinate(square):
    board_x = (square.row - 3.5)*square_sidelength#*1.05
    board_y = -(square.col - 3.5)*square_sidelength#*1.1

    dx = board_x*math.cos(board_yaw) - board_y*math.sin(board_yaw)
    dy = board_y*math.cos(board_yaw) + board_x*math.sin(board_yaw)
    result = Point(board_centerpoint.x + dx, board_centerpoint.y + dy, board_centerpoint.z)
    return result

def pickup_piece(square):
    hover_over_square(square)
    print('now over piece to pickup')
    go_down_to_square(square)
    rospy.logwarn('now about to grip piece')
    close_gripper()
    print('piece gripped')
    hover_over_square(square)
    print('piece picked up')

def set_piece(square):
    hover_over_square(square)
    print('now over location to set piece')
    go_down_to_square(square)
    print('now about to release piece')
    open_gripper()
    print('piece set down')
    hover_over_square(square)
    print('now backed away from piece')

def execute_pose(pose):
    posestamped = PoseStamped()
    posestamped.header.stamp = rospy.Time.now()
    posestamped.header.frame_id = 'torso_lift_link'
    posestamped.pose = pose
    goal = cart_interp.msg.CartesianArmServerGoal(setpoint = posestamped)
    arm_client.send_goal(goal)
    arm_client.wait_for_result(timeout=rospy.Duration(5.0))

def hover_over_square(square):
    global elevated_altitude, orientation
    hover_over = Pose()
    hover_over.position = square_to_coordinate(square)
    hover_over.position.z += elevated_altitude
    hover_over.orientation = orientation
    execute_pose(hover_over)

def go_down_to_square(square):
    global boardlevel_altitude, orientation
    down_to_square = Pose()
    down_to_square.position = square_to_coordinate(square)
    down_to_square.position.z += boardlevel_altitude
    down_to_square.orientation = orientation
    execute_pose(down_to_square)

def close_gripper():
    close_grip = Pr2GripperCommandGoal()
    close_grip.command.position = 0.00
    close_grip.command.max_effort = 100.0
    gripper_client.send_goal(close_grip)
    gripper_client.wait_for_result()

def open_gripper():
    open_grip = Pr2GripperCommandGoal()
    open_grip.command.position = 0.04
    open_grip.command.max_effort = -1.0
    gripper_client.send_goal(open_grip)
    gripper_client.wait_for_result()
    
def move_piece(origin, destination):
    pickup_piece(origin)
    set_piece(destination)    

def update_chessboard(board_pose):
    global tflistener, board_centerpoint, board_yaw, latch_chessboard, point_pub, square_Sidelength

    if latch_chessboard:
        rospy.logdebug('chessboard latched')
        return

    try:
        transformed_board_pose = tflistener.transformPose('torso_lift_link', board_pose)
    except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException):
        return

    q = (transformed_board_pose.pose.orientation.x, transformed_board_pose.pose.orientation.y, transformed_board_pose.pose.orientation.z, transformed_board_pose.pose.orientation.w)
    q = tf.transformations.quaternion_inverse(q)
    P_in = (3*square_sidelength, 3*square_sidelength, 0, 0)
    conj_q = tf.transformations.quaternion_conjugate(q)
    
    P_out = tf.transformations.quaternion_multiply(conj_q, tf.transformations.quaternion_multiply(P_in, q))

    board_centerpoint = Point(P_out[0]+transformed_board_pose.pose.position.x, P_out[1]+transformed_board_pose.pose.position.y, transformed_board_pose.pose.position.z)

    board_yaw = math.atan2(P_out[1], P_out[0])

    if (board_yaw < 0):
        board_yaw+=2*math.pi
    board_yaw = (board_yaw+(math.pi/4))%(math.pi/2)
    if (board_yaw > (math.pi/4)):
        board_yaw-=(math.pi/2)

    centerpoint_stamped = PoseStamped()
    centerpoint_stamped.header = transformed_board_pose.header
    centerpoint_stamped.pose.orientation = transformed_board_pose.pose.orientation
    centerpoint_stamped.pose.position = square_to_coordinate(Square(7, 7)) #board_centerpoint
    point_pub.publish(centerpoint_stamped)

def setup():
    global tflistener, board_centerpoint, arm_client, gripper_client, square_sidelength, out_of_sight, boardlevel_altitude, elevated_altitude, orientation, latch_chessboard, point_pub
    latch_chessboard = False

    point_pub = rospy.Publisher("board_center", PoseStamped)

    square_sidelength = 0.0339328125
    board_centerpoint = None

    orientation = Quaternion(-0.00403457514937, -0.780539478508, 0.00205700227215, -0.625090084256)
    boardlevel_altitude = 0.04
    elevated_altitude = 0.15

    out_of_sight = Pose()
    out_of_sight.position = Point(0.842470855969, -0.185501577783, 0.315879531841)
    out_of_sight.orientation = Quaternion(-0.00403457514937, -0.580539478508, 0.00205700227215, 0.814219506545)

    tflistener = tf.TransformListener()
    rospy.Subscriber("/checkerdetector/board_pose", geometry_msgs.msg.PoseStamped, update_chessboard)
    arm_client = actionlib.SimpleActionClient('r_cart_action_server', cart_interp.msg.CartesianArmServerAction)
    arm_client.wait_for_server()
    gripper_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', pr2_controllers_msgs.msg.Pr2GripperCommandAction)
    gripper_client.wait_for_server()

def main():
    global out_of_the_way
    rospy.init_node('chessbot')
    setup()
    execute_pose(out_of_sight)
    open_gripper()
    rate=rospy.Rate(1.0)
    #wait until we know the chessboard's position
    while (board_centerpoint is None):
        rate.sleep()
    print('chessboard located')
    server = Chessbot(rospy.get_name())
    rospy.spin()

if __name__ == '__main__':
    main()
