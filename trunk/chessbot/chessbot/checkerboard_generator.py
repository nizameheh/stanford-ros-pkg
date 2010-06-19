def main():
    squares_wide = 8
    squares_long= 8
    square_sidelength = 0.0339328125
    board_thickness = 0.0035
    square_type_1_laser_retro = 3000
    square_type_1_material = 'Gazebo/Red'
    square_type_2_laser_retro = 1000
    square_type_2_material = 'Gazebo/Black'
    
    print('<?xml version="1.0" ?>')
    print('<model:physical name="checkerboard_model">')
    print('\t<xyz>   0    0    0 </xyz>')
    print('\t<rpy>   0    0    0</rpy>')
    print('\t<static>true</static>')
    print('\t<body:box name="checkerboard_body">')
    for i in range(squares_wide):
        for j in range(squares_long):
            print('\t\t<geom:box name="checker_' + str(i) + str(j) + '_geom">')
            print('\t\t\t<xyz>' + str(square_sidelength*i) + ' ' + str(square_sidelength*j) + ' 0.0</xyz>')
            print('\t\t\t<size>' + str(square_sidelength) + ' ' + str(square_sidelength) + ' ' +  str(board_thickness)+'</size>')
            if ((i + j) %  2 == 0):
                print('\t\t\t<laserRetro>' + str(square_type_1_laser_retro) + '</laserRetro>')
            else:
                print('\t\t\t<laserRetro>' + str(square_type_2_laser_retro) + '</laserRetro>')
            print('\t\t\t<kp>1000000.0</kp>')
            print('\t\t\t<kd>1.0</kd>')
            print('\t\t\t<mesh>default</mesh>')
            print('\t\t\t<mass> 0.01</mass>')
            print('\t\t\t<visual>')
            print('\t\t\t\t<size>' + str(square_sidelength) + ' ' + str(square_sidelength) + ' ' +  str(board_thickness)+'</size>')
            if ((i + j)%2 == 0):
                print('\t\t\t\t<material>' + square_type_1_material + '</material>')
            else:
                print('\t\t\t\t<material>' + square_type_2_material + '</material>')
            print('\t\t\t\t<mesh>unit_box</mesh>')
            print('\t\t\t</visual>')
            print('\t\t</geom:box>')
    print('\t</body:box>')
    print('</model:physical>')

if __name__ == '__main__':
    main()
