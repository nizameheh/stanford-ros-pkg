import roslib
roslib.load_manifest('chessbot')
import rospy
import actionlib
import chessbot.msg
import pexpect

chessboard = []
chessboard.append(['R','N','B','Q','K','B','N','R'])
chessboard.append(['P','P','P','P','P','P','P','P'])
chessboard.append([' ',' ',' ',' ',' ',' ',' ',' '])
chessboard.append([' ',' ',' ',' ',' ',' ',' ',' '])
chessboard.append([' ',' ',' ',' ',' ',' ',' ',' '])
chessboard.append([' ',' ',' ',' ',' ',' ',' ',' '])
chessboard.append(['p','p','p','p','p','p','p','p'])
chessboard.append(['r','n','b','q','k','b','n','r'])

def isEmpty(file_letter, rank_number):
    return getPiece(file_letter, rank_number) is ' '


def setPiece(file_letter, rank_number, piece):
    chessboard[rank_number-1][col_number_from_file_letter(file_letter)] = piece


def getPiece(file_letter, rank_number):
    return(chessboard[rank_number-1][col_number_from_file_letter(file_letter)])


def col_number_from_file_letter(file_letter):
    return ord(file_letter) - ord('a')


def movePiece(origin_file, origin_rank, destination_file, destination_rank, capture):
    global chessbot_client
    setPiece(destination_file, destination_rank, getPiece(origin_file, origin_rank))
    setPiece(origin_file, origin_rank, ' ')
    robot_move = chessbot.msg.ChessbotGoal(origin_rank - 1, col_number_from_file_letter(origin_file), destination_rank - 1, col_number_from_file_letter(destination_file), capture)
    chessbot_client.send_goal(robot_move)
    chessbot_client.wait_for_result()
    
    
def makeMove(move):
    origin_file = move[0]
    origin_rank  = int(move[1])
    destination_file = move[2]
    destination_rank = int(move[3])
    if (move[0:4] == 'e1g1') and (getPiece('e', 1) == 'K'):
        #white castling short
        movePiece('e', 1, 'g', 1, False)
        movePiece('h', 1, 'f', 1, False)
    elif (move[0:4] == 'e1c1') and (getPiece('e', 1) == 'K'):
        #white castling long
        movePiece('e', 1, 'c', 1, False)
        movePiece('a', 1, 'd', 1, False)
    elif (move[0:4] == 'e8g8') and (getPiece('e', 5) == 'k'):
        #black castling short
        movePiece('e', 8, 'g', 8, False)
        movePiece('h', 8, 'f', 8, False)
    elif (move[0:4] == 'e8c8') and (getPiece('e', 5) == 'k'):
        #black castling long
        movePiece('e', 8, 'c', 8, False)
        movePiece('a', 8, 'd', 8, False)
    else:
        movePiece(origin_file, origin_rank, destination_file, destination_rank, isEmpty(destination_file, destination_rank))
        if len(move) == 5:
            #pawn promotion
            if move[4].upper() == move[4]:
                setPiece(destination_file, destination_rank, move[4].upper())
            else:
                setPiece(destination_file, destination_rank, move[4].lower())


def main():
    global chessbot_client
    REGEX_MOVE = '([a-h][1-8][a-h][1-8][RrNnBbQq(\r\n)])'
    REGEX_RESULT = '((0-1)|(1-0)|(1/2-1/2)) {[a-zA-Z ]*}'
    
    rospy.init_node('gnuchessmaster')
    chessbot_client = actionlib.SimpleActionClient('chessbot', chessbot.msg.ChessbotAction)
    chessbot_client.wait_for_server()
    gnuchess = pexpect.spawn('/usr/games/gnuchess -x', maxread=1)
    i = 0

    while (gnuchess.isalive()):
        gnuchess.sendline('go')
        if (i % 2 == 0):
            print 'WHITE:'
        else:
            print 'BLACK:'
        gnuchess.expect ('My move is')
        print '', gnuchess.after
        index  = gnuchess.expect ([REGEX_MOVE, REGEX_RESULT])
        if (index == 1):
            break
            
        move = gnuchess.after.strip()
        makeMove(move)
        
        print gnuchess.after

        print('   a    b    c    d    e    f    g    h')
        for j in range(8,0,-1):
            print str(j)+str(chessboard[j-1])+str(j)
        print('   a    b    c    d    e    f    g    h')
        i += 1        


if __name__=='__main__':
    main()
