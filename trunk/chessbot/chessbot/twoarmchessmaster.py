import roslib
roslib.load_manifest('chessbot')
import rospy
import actionlib
import chessbot.msg

def main():
    rospy.init_node('chessmaster')
    right_chessbot_client = actionlib.SimpleActionClient('right_chessbot/chessbot', chessbot.msg.ChessbotAction)
    right_chessbot_client.wait_for_server()
    left_chessbot_client = actionlib.SimpleActionClient('left_chessbot/chessbot', chessbot.msg.ChessbotAction)
    left_chessbot_client.wait_for_server()
    move = chessbot.msg.ChessbotGoal(0, 0, 7, 7, False)
    right_chessbot_client.send_goal(move)
    right_chessbot_client.wait_for_result()
    move = chessbot.msg.ChessbotGoal(7, 7, 0, 0, False)
    left_chessbot_client.send_goal(move)
    left_chessbot_client.wait_for_result()

if __name__ == '__main__':
    main()
