import math

def main():
    square_sidelength = 0.0339328125
    squareA1_x = 0.315
    squareA1_y = -0.15 + 7*square_sidelength
    squareA1_z = 0.41
    board_yaw = 0.0
    
    print('<launch>')
    for i in range(2):
        for j in range(8):
            dx = (i * math.cos(board_yaw) + j * math.sin(board_yaw)) * square_sidelength
            dy = ((-j) * math.cos(board_yaw)+i * math.sin(board_yaw)) * square_sidelength
            print('\t<node name="spawn_chesspiece'+str(i)+str(j)+'" pkg="gazebo_tools" type="gazebo_model" args="-f $(find chessbot)/objects/whitechesspiece.urdf -x '+str(squareA1_x + dx)+' -y '+str(squareA1_y + dy)+' -z '+str(squareA1_z)+' -R 0.0 -P 0.0 -Y '+str(board_yaw)+' spawn chesspiece'+str(i)+str(j)+'" respawn="false" output="screen" />')
    for i in range(6, 8):
        for j in range(8):
            dx = (i * math.cos(board_yaw) + j * math.sin(board_yaw)) * square_sidelength
            dy = ((-j) * math.cos(board_yaw)+i * math.sin(board_yaw)) * square_sidelength
            print('\t<node name="spawn_chesspiece'+str(i)+str(j)+'" pkg="gazebo_tools" type="gazebo_model" args="-f $(find chessbot)/objects/blackchesspiece.urdf -x '+str(squareA1_x + dx)+' -y '+str(squareA1_y + dy)+' -z '+str(squareA1_z)+' -R 0.0 -P 0.0 -Y '+str(board_yaw)+' spawn chesspiece'+str(i)+str(j)+'" respawn="false" output="screen" />')
    print('</launch>')
    

if __name__ == '__main__':
    main()
