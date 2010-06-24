#!/usr/bin/env python

import roslib
roslib.load_manifest('chessbot')
import rospy
import math
import actionlib
from  cart_interp.msg import *
from chessbot.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *
import tf

class Chessbot:
    def __init__(self, name):
        self.setup()
        self.execute_pose(self.out_of_sight)
        self.open_gripper()
        rate=rospy.Rate(1.0)
        #wait until the chessboard's position is known before advertising service
        while (self.board_centerpoint is None):
            rate.sleep()

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ChessbotAction, execute_cb=self.execute)

    def execute(self, goal):
        self.latch_chessboard = True
        destination = (goal.destination_row, goal.destination_col)
        origin = (goal.origin_row, goal.origin_col)
        if goal.capture:
            move_piece(destination, (-2, -2))
        self.move_piece(origin, destination)
        self._as.set_succeeded()
        self.latch_chessboard = False

    def square_to_coordinate(self, square):
        row, col = square
        
        #Using offset 3.5 sets (0, 0) to be the middle of the lower left square
        board_x = (row - 3.5)*self.square_sidelength
        board_y = -(col - 3.5)*self.square_sidelength
        
        dx = board_x*math.cos(self.board_yaw) - board_y*math.sin(self.board_yaw)
        dy = board_y*math.cos(self.board_yaw) + board_x*math.sin(self.board_yaw)
        result = Point()
        result.x = self.board_centerpoint.x + dx
        result.y = self.board_centerpoint.y + dy
        result.z = self.board_centerpoint.z
        return result

    def pickup_piece(self, square):
        self.hover_over_square(square)
        rospy.sleep(2.0)
        self.go_down_to_square(square, False)
        self.close_gripper()
        self.hover_over_square(square)

    def set_piece(self, square):
        self.hover_over_square(square)
        self.go_down_to_square(square, True)
        rospy.sleep(2.0)
        self.open_gripper()
        self.hover_over_square(square)
        
    def execute_pose(self, pose):
        posestamped = PoseStamped()
        posestamped.header.stamp = rospy.Time.now()
        posestamped.header.frame_id = 'torso_lift_link'
        posestamped.pose = pose
        goal = CartesianArmServerGoal(setpoint=posestamped)
        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(timeout=rospy.Duration(5.0))

    def hover_over_square(self, square):
        hover_over = Pose()
        hover_over.position = self.square_to_coordinate(square)
        hover_over.position.z += self.elevated_altitude
        hover_over.orientation = self.orientation
        self.execute_pose(hover_over)

    def go_down_to_square(self, square, compliance):
        down_to_square = Pose()
        down_to_square.position = self.square_to_coordinate(square)
        down_to_square.position.z += self.boardlevel_altitude
        if compliance:
            down_to_square.position.z += 0.01
        down_to_square.orientation = self.orientation
        self.execute_pose(down_to_square)

    def close_gripper(self):
        close_grip = Pr2GripperCommandGoal()
        close_grip.command.position = 0.00
        close_grip.command.max_effort = self.gripper_close_max_effort
        self.gripper_client.send_goal(close_grip)
        self.gripper_client.wait_for_result()

    def open_gripper(self):
        open_grip = Pr2GripperCommandGoal()
        open_grip.command.position = self.gripper_open_position
        open_grip.command.max_effort = -1.0
        self.gripper_client.send_goal(open_grip)
        self.gripper_client.wait_for_result()
    
    def move_piece(self, origin, destination):
        self.pickup_piece(origin)
        self.set_piece(destination)    

    def update_chessboard(self, board_pose):        
        if self.latch_chessboard:
            rospy.logdebug('Chessboard position latched')
            return
        
        try:
            transformed_board_pose = self.tflistener.transformPose('torso_lift_link', board_pose)
        except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException):
            return
            
        #transform board centerpoint to  appropriate frame
        rot = transformed_board_pose.pose.orientation
        q = (rot.x, rot.y, rot.z, rot.w)
        q = tf.transformations.quaternion_inverse(q)
        P_in = (3 * self.square_sidelength, 3 * self.square_sidelength, 0, 0)
        conj_q = tf.transformations.quaternion_conjugate(q)
        product = tf.transformations.quaternion_multiply(P_in, q)
        P_out = tf.transformations.quaternion_multiply(conj_q, product)
        trans = transformed_board_pose.pose.position
        self.board_centerpoint = Point()
        self.board_centerpoint.x = P_out[0] + trans.x
        self.board_centerpoint.y = P_out[1] + trans.y
        self.board_centerpoint.z = P_out[2] + trans.z

        #Define yaw on (-pi/4, pi/4), 0 is when board sides are parallel with axes.
        self.board_yaw = (math.atan2(P_out[1], P_out[0]) % (math.pi/2))-(math.pi/4)
            
    def setup(self):
        self.latch_chessboard = False
        self.square_sidelength = rospy.get_param('square_sidelength')
        self.boardlevel_altitude = rospy.get_param('boardlevel_altitude')
        self.elevated_altitude = rospy.get_param('elevated_altitude')
        self.out_of_sight = Pose()
        self.out_of_sight.position.x = rospy.get_param('out_of_sight_pose/position/x')
        self.out_of_sight.position.y = rospy.get_param('out_of_sight_pose/position/y')
        self.out_of_sight.position.z = rospy.get_param('out_of_sight_pose/position/z')
        self.out_of_sight.orientation.x = rospy.get_param('out_of_sight_pose/orientation/x')
        self.out_of_sight.orientation.y = rospy.get_param('out_of_sight_pose/orientation/y')
        self.out_of_sight.orientation.w = rospy.get_param('out_of_sight_pose/orientation/w')
        self.out_of_sight.orientation.z = rospy.get_param('out_of_sight_pose/orientation/z')
        self.orientation = Quaternion()
        self.orientation.x = rospy.get_param('gripper_orientation/x')
        self.orientation.y = rospy.get_param('gripper_orientation/y')
        self.orientation.z = rospy.get_param('gripper_orientation/z')
        self.orientation.w = rospy.get_param('gripper_orientation/w')
        self.gripper_open_position = rospy.get_param('gripper_open_position')
        self.gripper_close_max_effort = rospy.get_param('gripper_close_max_effort')
        self.root_frame = rospy.get_param('root_frame')
        self.board_centerpoint = None
        self.tflistener = tf.TransformListener()
        rospy.Subscriber("chessboard_detector/board_pose", PoseStamped, self.update_chessboard)
        self.arm_client = actionlib.SimpleActionClient('cartesian_arm_server', CartesianArmServerAction)
        self.arm_client.wait_for_server()
        self.gripper_client = actionlib.SimpleActionClient('gripper_server', Pr2GripperCommandAction)
        self.gripper_client.wait_for_server()
        
def main():
    rospy.init_node('chessbot')
    server = Chessbot(rospy.get_name())
    rospy.spin()

if __name__ == '__main__':
    main()
