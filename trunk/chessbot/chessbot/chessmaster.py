import roslib
roslib.load_manifest('chessbot')
import rospy
import actionlib
import chessbot.msg

def main():
    rospy.init_node('chessmaster')
    chessbot_client = actionlib.SimpleActionClient('chessbot', chessbot.msg.ChessbotAction)
    chessbot_client.wait_for_server()
    move = chessbot.msg.ChessbotGoal(1, 1, 6, 6, False)
    chessbot_client.send_goal(move)
    chessbot_client.wait_for_result()

if __name__ == '__main__':
    main()
