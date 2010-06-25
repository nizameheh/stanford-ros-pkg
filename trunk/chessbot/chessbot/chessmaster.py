import roslib
roslib.load_manifest('chessbot')
import rospy
import actionlib
import chessbot.msg

def main():
    rospy.init_node('chessmaster')
    chessbot_client = actionlib.SimpleActionClient('chessbot/chessbot', chessbot.msg.ChessbotAction)
    chessbot_client.wait_for_server()
    move = chessbot.msg.ChessbotGoal(3, 3, 0, 0, False)
    chessbot_client.send_goal(move)
    chessbot_client.wait_for_result()

if __name__ == '__main__':
    main()
