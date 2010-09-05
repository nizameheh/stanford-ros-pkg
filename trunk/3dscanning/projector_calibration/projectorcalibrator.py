#!/usr/bin/env python

import roslib
roslib.load_manifest('projector_calibration')
import rospy
import cv
import projector.srv
import sensor_msgs.msg
import sensor_msgs.srv
from cv_bridge import CvBridge, CvBridgeError
import threading
import image_geometry
import math

class ProjectorCalibrator:
    def __init__(self):
        self.printed_chessboard_corners_x = rospy.get_param('~printed_chessboard_corners_x')
        self.printed_chessboard_corners_y = rospy.get_param('~printed_chessboard_corners_y')
        self.printed_chessboard_spacing = rospy.get_param('~printed_chessboard_spacing')

        self.bridge = CvBridge()
        self.mutex = threading.RLock()
        self.image_update_flag = threading.Event()

        rospy.Subscriber("image_stream", sensor_msgs.msg.Image, self.update_image)
        rospy.loginfo("Waiting for camera info...")
        self.camera_info = rospy.wait_for_message('camera_info', sensor_msgs.msg.CameraInfo)
        rospy.loginfo("Camera info received.")

        rospy.loginfo("Waiting for projector info service...")
        rospy.wait_for_service('get_projector_info')
        rospy.loginfo("Projector info service found.")
        projector_info_service = rospy.ServiceProxy('get_projector_info', projector.srv.GetProjectorInfo)
        
        rospy.loginfo("Waiting for projection setting service...")
        rospy.wait_for_service('set_projection')
        rospy.loginfo("Projection setting service found.")
        self.set_projection = rospy.ServiceProxy('set_projection', projector.srv.SetProjection)

        rospy.loginfo("Waiting for projector info setting service...")
        rospy.wait_for_service('set_projector_info')
        rospy.loginfo("Projection setting service found.")
        self.set_projector_info = rospy.ServiceProxy('set_projector_info', sensor_msgs.srv.SetCameraInfo)

        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        self.projector_info = projector_info_service().projector_info

        self.blank_projection = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_8UC1)
        cv.SetZero(self.blank_projection)        

        self.number_of_scenes = 0
        self.object_points = []
        self.image_points = []


    def set_projected_chessboard(self, corners_x, corners_y, spacing, horizontal_justify, vertical_justify):
        self.projected_chessboard_corners_x = corners_x
        self.projected_chessboard_corners_y = corners_y
        self.projected_chessboard_spacing = spacing
  
        if horizontal_justify is 0:
            self.chessboard_upper_left_corner_x = 0
        elif horizontal_justify is 1:
            self.chessboard_upper_left_corner_x = (self.projector_info.width - (self.projected_chessboard_spacing * (self.projected_chessboard_corners_x + 1))) // 2
        elif horizontal_justify is 2:
            self.chessboard_upper_left_corner_x = (self.projector_info.width - (self.projected_chessboard_spacing * (self.projected_chessboard_corners_x + 1)))

        if vertical_justify is 0:
            self.chessboard_upper_left_corner_y = 0
        elif vertical_justify is 1:
            self.chessboard_upper_left_corner_y = (self.projector_info.height - (self.projected_chessboard_spacing * (self.projected_chessboard_corners_y + 1))) // 2
        elif vertical_justify is 2:
            self.chessboard_upper_left_corner_y = (self.projector_info.height - (self.projected_chessboard_spacing * (self.projected_chessboard_corners_y + 1)))
        
        self.positive_chessboard_projection = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_8UC1)
        cv.Set(self.positive_chessboard_projection, 255)
        for row in range(self.projected_chessboard_corners_y + 1):
            for col in range(self.projected_chessboard_corners_x + 1):
                if (row + col) % 2 == 0:
                    cv.Rectangle(self.positive_chessboard_projection,
                                 (self.chessboard_upper_left_corner_x + (col * self.projected_chessboard_spacing), self.chessboard_upper_left_corner_y + (row * self.projected_chessboard_spacing)),
                                 (self.chessboard_upper_left_corner_x + ((col + 1) * self.projected_chessboard_spacing) - 1, self.chessboard_upper_left_corner_y + ((row + 1) * self.projected_chessboard_spacing) - 1),
                                 0,
                                 cv.CV_FILLED)

        self.negative_chessboard_projection = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_8UC1)
        cv.Not(self.positive_chessboard_projection, self.negative_chessboard_projection)

    def update_image(self, imagemsg):
        with self.mutex:
            if not self.image_update_flag.is_set():
                self.latest_image = self.bridge.imgmsg_to_cv(imagemsg, "mono8")
                self.image_update_flag.set()
                self.image_update_flag.clear()

    def get_next_image(self):
        with self.mutex:
            self.image_update_flag.clear()
        self.image_update_flag.wait()
        with self.mutex:
            return self.latest_image
                    
    def get_picture_of_projection(self, projection):
        image_message = self.bridge.cv_to_imgmsg(projection, encoding="mono8")
        self.set_projection(image_message)
        return self.get_next_image()
    
    def preview_chessboard(self):
        image_message = self.bridge.cv_to_imgmsg(self.negative_chessboard_projection, encoding="mono8")
        self.set_projection(image_message)

    def get_scene_image_points(self):
        scene_image_points = cv.CreateMat(1, self.projected_chessboard_corners_x * self.projected_chessboard_corners_y, cv.CV_32FC2)
        for row in range(self.projected_chessboard_corners_y):
            for col in range(self.projected_chessboard_corners_x):
                index = (row * self.projected_chessboard_corners_x) + col
                point = (self.chessboard_upper_left_corner_x + (self.projected_chessboard_spacing * (col + 1)), 
                         self.chessboard_upper_left_corner_y + (self.projected_chessboard_spacing * (row + 1)))
                scene_image_points[0, index] = point
        return scene_image_points            

    def commit(self):
        response = self.set_projector_info(self.projector_info)
        return response.success        

    def calibrate(self):
        image_points_mat = concatenate_mats(self.image_points)
        object_points_mat = concatenate_mats(self.object_points)
        print "Image Points:"
        for row in range(image_points_mat.height):
            for col in range(image_points_mat.width):
                print image_points_mat[row, col]
            print

        print "Object Points:"
        for row in range(object_points_mat.height):
            for col in range(object_points_mat.width):
                print object_points_mat[row, col]
            print
        
        point_counts_mat = cv.CreateMat(1, self.number_of_scenes, cv.CV_32SC1)
        for i in range(self.number_of_scenes):
            point_counts_mat[0, i] = self.image_points[i].width
        intrinsics = cv.CreateMat(3, 3, cv.CV_32FC1)
        distortion = cv.CreateMat(4, 1, cv.CV_32FC1)
        cv.SetZero(intrinsics)
        cv.SetZero(distortion)
        size = (self.projector_info.width, self.projector_info.height)
        tvecs = cv.CreateMat(self.number_of_scenes, 3,cv.CV_32FC1)
        rvecs = cv.CreateMat(self.number_of_scenes, 3, cv.CV_32FC1)
        cv.CalibrateCamera2(object_points_mat, image_points_mat, point_counts_mat,
                   size, intrinsics,
                   distortion,
                   rvecs,
                   tvecs,
                   flags = 0)

        
        R = cv.CreateMat(3, 3, cv.CV_32FC1)
        P = cv.CreateMat(3, 4, cv.CV_32FC1)
        cv.SetIdentity(R)
        cv.SetZero(P)
        cv.Copy(intrinsics, cv.GetSubRect(P, (0, 0, 3, 3)))

        self.projector_info = create_msg(size, distortion, intrinsics, R, P)
        rospy.loginfo(self.projector_info)
        
        for col in range(3):
            for row in range(self.number_of_scenes):
                print tvecs[row, col],
            print

    def capture(self):
        blank = self.get_picture_of_projection(self.blank_projection)
        positive = self.get_picture_of_projection(self.positive_chessboard_projection)
        negative = self.get_picture_of_projection(self.negative_chessboard_projection)

        difference = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
        cv.Sub(positive, negative, difference)

        cv.NamedWindow("blank", flags=0)
        cv.ShowImage("blank", blank)
        cv.WaitKey(300)

        cv.NamedWindow("difference", flags=0)
        cv.ShowImage("difference", difference)
        cv.WaitKey(300)
        
        camera_image_points, camera_object_points = detect_chessboard(blank, self.printed_chessboard_corners_x, self.printed_chessboard_corners_y, self.printed_chessboard_spacing, self.printed_chessboard_spacing)
        if camera_image_points is None:
            return False
        cv.UndistortPoints(camera_image_points, camera_image_points, self.camera_model.intrinsicMatrix(), self.camera_model.distortionCoeffs())
        homography = cv.CreateMat(3, 3, cv.CV_32FC1)
        cv.FindHomography(camera_image_points, camera_object_points, homography)   
        object_points, dummy = detect_chessboard(difference, self.projected_chessboard_corners_x, self.projected_chessboard_corners_y, None, None)
        if object_points is None:
            return False
        cv.UndistortPoints(object_points, object_points, self.camera_model.intrinsicMatrix(), self.camera_model.distortionCoeffs())
        
        cv.PerspectiveTransform(object_points, object_points, homography)

        object_points_3d = cv.CreateMat(1, self.projected_chessboard_corners_x * self.projected_chessboard_corners_y, cv.CV_32FC3)
        
        x = cv.CreateMat(1, self.projected_chessboard_corners_x * self.projected_chessboard_corners_y, cv.CV_32FC1)
        y = cv.CreateMat(1, self.projected_chessboard_corners_x * self.projected_chessboard_corners_y, cv.CV_32FC1)
        cv.Split(object_points, x, y, None, None)
        z = cv.CreateMat(1, self.projected_chessboard_corners_x * self.projected_chessboard_corners_y, cv.CV_32FC1)
        cv.SetZero(z)

        cv.Merge(x, y, z, None, object_points_3d)

        self.object_points.append(object_points_3d)
        self.image_points.append(self.get_scene_image_points())
        self.number_of_scenes += 1

        return True

def cvmat_iterator(cvmat):
    for i in range(cvmat.rows):
        for j in range(cvmat.cols):
            yield cvmat[i,j]
            
def create_msg(size, d, k, r, p):
    msg = sensor_msgs.msg.CameraInfo()
    (msg.width, msg.height) = size
    msg.D = [d[i,0] for i in range(d.rows)]
    while len(msg.D)<5:
        msg.D.append(0)
    msg.K = list(cvmat_iterator(k))
    msg.R = list(cvmat_iterator(r))
    msg.P = list(cvmat_iterator(p))
    return msg

def concatenate_mats(list_of_mats):
    rows = list_of_mats[0].height
    cols = 0;
    for i in range(len(list_of_mats)):
        cols += list_of_mats[i].width
    destination = cv.CreateMat(rows, cols, list_of_mats[0].type)
    destination_col = 0
    for i in range(len(list_of_mats)):
        for row in range(rows):
            for source_col in range(list_of_mats[i].width):
                destination[row, destination_col + source_col] = list_of_mats[i][row, source_col]
        destination_col += list_of_mats[i].width
    return destination

def get_board_corners(corners, corners_x, corners_y):
    return (corners[0], corners[corners_x  - 1], 
        corners[(corners_y - 1) * corners_x], corners[len(corners) - 1])

#This function is stolen outright from Eiten Marder-Eppstein's checkerboard_pose package, modified only to remove its scaling features
def detect_chessboard(image, corners_x, corners_y, spacing_x, spacing_y):
    #Here, we'll actually call the openCV detector    
    found, corners = cv.FindChessboardCorners(image, (corners_x, corners_y), cv.CV_CALIB_CB_ADAPTIVE_THRESH)

    if found:
      board_corners = get_board_corners(corners, corners_x, corners_y)
      
      #find the perimeter of the checkerboard
      perimeter = 0.0
      for i in range(len(board_corners)):
        next = (i + 1) % 4
        xdiff = board_corners[i][0] - board_corners[next][0]
        ydiff = board_corners[i][1] - board_corners[next][1]
        perimeter += math.sqrt(xdiff * xdiff + ydiff * ydiff)

      #estimate the square size in pixels
      square_size = perimeter / ((corners_x - 1 + corners_y - 1) * 2)
      radius = int(square_size * 0.5 + 0.5)

      corners = cv.FindCornerSubPix(image, corners, (radius, radius), (-1, -1), (cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 30, 0.1))

      #uncomment to debug chessboard detection
      print 'Chessboard found'
      #cv.DrawChessboardCorners(image_scaled, (corners_x, corners_y), corners, 1)
      #cv.NamedWindow("image_scaled")
      #cv.ShowImage("image_scaled", image_scaled)
      #cv.WaitKey(600)

      object_points = None

      #we'll also generate the object points if the user has specified spacing
      if spacing_x != None and spacing_y != None:
          object_points = cv.CreateMat(1, corners_x * corners_y, cv.CV_32FC3)

          for y in range(corners_y):
              for x in range(corners_x):
                  object_points[0, y*corners_x + x] = (x * spacing_x, y * spacing_y, 0.0)
                  
      #not sure why opencv functions return non opencv compatible datatypes... but they do so we'll convert
      corners_cv = cv.CreateMat(1, corners_x * corners_y, cv.CV_32FC2)
      for i in range(corners_x * corners_y):
          corners_cv[0, i] = (corners[i][0], corners[i][1])

      return (corners_cv, object_points)

    else:
      #cv.NamedWindow("image_scaled")
      #cv.ShowImage("image_scaled", image_scaled)
      #cv.WaitKey(600)
      rospy.logwarn("Didn't find checkerboard")
      return (None, None)

def main():
    rospy.init_node('projector_calibrator')
    projector_calibrator = ProjectorCalibrator()

    while not rospy.is_shutdown():
        response = raw_input("Define the projected chessboard:")
        response = response.split()
        if len(response) == 4:
            corners = response[0].split('x')
            if len(corners) == 2 and response[2] in ('0', '1', '2') and response[3] in ('0', '1', '2'):
                try:
                    projector_calibrator.set_projected_chessboard(int(corners[0]), int(corners[1]), int(response[1]), int(response[2]), int(response[3]))
                    projector_calibrator.preview_chessboard()
                    break;
                except ValueError:
                    pass
    while not rospy.is_shutdown():
        print str(projector_calibrator.number_of_scenes) + " scenes captured so far."
        response = raw_input('Press C to capture, N to calibrate with the collected data, or R to redefine the projected chessboard: ')
        if response.upper() == 'C':
            if projector_calibrator.capture() is False:
                print "Capture Error."
        elif response.upper() == 'N':
            projector_calibrator.calibrate()
            response = raw_input('Press C to commit this calibration, or N to continue capturing: ')
            if response.upper() == 'C':
                if projector_calibrator.commit() is True:
                    print 'Projector info successfully set'
                    break;
                else:
                    print 'Projector info setting failed'
        elif response.upper() == 'R':
            while not rospy.is_shutdown():
                response = raw_input("Define the projected chessboard:")
                response = response.split()
                if len(response) == 4:
                    corners = response[0].split('x')
                    if len(corners) == 2 and response[2] in ('0', '1', '2') and response[3] in ('0', '1', '2'):
                        try:
                            projector_calibrator.set_projected_chessboard(int(corners[0]), int(corners[1]), int(response[1]), int(response[2]), int(response[3]))
                            projector_calibrator.preview_chessboard()
                            break;
                        except ValueError:
                            pass
        else:
            print "Invalid response."
    

if __name__ == '__main__':
    main()
