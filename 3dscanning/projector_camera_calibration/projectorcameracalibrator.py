#!/usr/bin/env python

import roslib
roslib.load_manifest('projector_camera_calibration')
import rospy
import cv
import projector.srv
import sensor_msgs.msg
import sensor_msgs.srv
from cv_bridge import CvBridge, CvBridgeError
import threading
import image_geometry
import math
import yaml

class ProjectorCameraCalibrator:
    def __init__(self):
        self.projected_chessboard_corners_x = rospy.get_param('~projected_chessboard_corners_x')
        self.projected_chessboard_corners_y = rospy.get_param('~projected_chessboard_corners_y')
        self.projected_chessboard_spacing = rospy.get_param('~projected_chessboard_spacing')

        self.printed_chessboard_corners_x = rospy.get_param('~printed_chessboard_corners_x')
        self.printed_chessboard_corners_y = rospy.get_param('~printed_chessboard_corners_y')
        self.printed_chessboard_spacing = rospy.get_param('~printed_chessboard_spacing')

        self.projector_camera_info_file_name = rospy.get_param('~projector_camera_info_file_name')

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
        self.projector_info = projector_info_service().projector_info
        
        rospy.loginfo("Waiting for projection setting service...")
        rospy.wait_for_service('set_projection')
        rospy.loginfo("Projection setting service found.")
        self.set_projection = rospy.ServiceProxy('set_projection', projector.srv.SetProjection)

        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        self.projector_model = image_geometry.PinholeCameraModel()
        self.projector_model.fromCameraInfo(self.projector_info)

        self.blank_projection = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_8UC1)
        cv.SetZero(self.blank_projection)

        # Generate chessboard projections
        self.chessboard_upper_left_corner_x = (self.projector_info.width - (self.projected_chessboard_spacing * (self.projected_chessboard_corners_x + 1))) // 2
        self.chessboard_upper_left_corner_y = (self.projector_info.height - (self.projected_chessboard_spacing * (self.projected_chessboard_corners_y + 1))) // 2
        
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
        
        self.object_points = []
        self.camera_image_points = []
        self.projector_image_points = []
        self.number_of_scenes = 0

        # Show negative chessboard so user can position printed chessboard for capture
        negative_chessboard_message = self.bridge.cv_to_imgmsg(self.negative_chessboard_projection, encoding="mono8")
        self.set_projection(negative_chessboard_message)        

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
    
    def get_projector_image_points(self):
        scene_image_points = cv.CreateMat(1, self.projected_chessboard_corners_x * self.projected_chessboard_corners_y, cv.CV_32FC2)
        for row in range(self.projected_chessboard_corners_y):
            for col in range(self.projected_chessboard_corners_x):
                index = (row * self.projected_chessboard_corners_x) + col
                point = (self.chessboard_upper_left_corner_x + (self.projected_chessboard_spacing * (col + 1)), 
                         self.chessboard_upper_left_corner_y + (self.projected_chessboard_spacing * (row + 1)))
                scene_image_points[0, index] = point
        return scene_image_points            

    def write_projector_camera_info(self):
        info_dict = {}
        info_dict['projector_to_camera_translation_vector'] = mat_to_list(self.projector_to_camera_translation_vector)
        info_dict['projector_to_camera_rotation_vector'] = mat_to_list(self.projector_to_camera_rotation_vector)
        output_stream = file(self.projector_camera_info_file_name, 'w')
        yaml.dump(info_dict, output_stream)
        rospy.loginfo("Projector camera info successfully written to " + self.projector_camera_info_file_name)

    def calibrate(self):
        self.projector_to_camera_translation_vector = cv.CreateMat(3, 1, cv.CV_32FC1)
        self.projector_to_camera_rotation_vector = cv.CreateMat(3, 1, cv.CV_32FC1)
        cv.SetZero(self.projector_to_camera_translation_vector)
        cv.SetZero(self.projector_to_camera_rotation_vector)
        for i in range(self.number_of_scenes):
            camera_tvec = cv.CreateMat(3, 1,cv.CV_32FC1)
            camera_rvec = cv.CreateMat(3, 1,cv.CV_32FC1)
            cv.FindExtrinsicCameraParams2(self.object_points[i],
                                          self.camera_image_points[i],
                                          self.camera_model.intrinsicMatrix(), 
                                          self.camera_model.distortionCoeffs(),
                                          camera_rvec,
                                          camera_tvec)

            print "Camera To Board Vector:"
            for row in range(camera_tvec.height):
                for col in range(camera_tvec.width):
                    print camera_tvec[row, col],
                print
            print
            projector_tvec = cv.CreateMat(3, 1,cv.CV_32FC1)
            projector_rvec = cv.CreateMat(3, 1,cv.CV_32FC1)
            cv.FindExtrinsicCameraParams2(self.object_points[i],
                                          self.projector_image_points[i],
                                          self.projector_model.intrinsicMatrix(), 
                                          self.projector_model.distortionCoeffs(),
                                          projector_rvec,
                                          projector_tvec)

            print "Projector To Board Vector:"
            for row in range(projector_tvec.height):
                for col in range(projector_tvec.width):
                    print projector_tvec[row, col],
                print
            print
            
            camera_rmat = cv.CreateMat(3, 3, cv.CV_32FC1)
            cv.Rodrigues2(camera_rvec, camera_rmat)

            projector_rmat = cv.CreateMat(3, 3, cv.CV_32FC1)
            cv.Rodrigues2(projector_rvec, projector_rmat)

            scene_projector_to_camera_rotation_matrix = cv.CreateMat(3, 3, cv.CV_32FC1)
            cv.GEMM(camera_rmat, projector_rmat, 1, None, 0, scene_projector_to_camera_rotation_matrix, cv.CV_GEMM_B_T)
            scene_projector_to_camera_rotation_vector = cv.CreateMat(3, 1, cv.CV_32FC1)
            for i in range(3):
                for j in range(3):
                    print scene_projector_to_camera_rotation_matrix[i, j],
                print
            print
            cv.Rodrigues2(scene_projector_to_camera_rotation_matrix, scene_projector_to_camera_rotation_vector)
            print "Scene Rotation Vector:"
            for row in range(scene_projector_to_camera_rotation_vector.height):
                for col in range(scene_projector_to_camera_rotation_vector.width):
                    print scene_projector_to_camera_rotation_vector[row, col],
                print
            print

            scene_projector_to_camera_translation_vector = cv.CreateMat(3, 1, cv.CV_32FC1)
            cv.GEMM(projector_rmat, projector_tvec, -1, None, 0, scene_projector_to_camera_translation_vector, cv.CV_GEMM_A_T)
            cv.GEMM(camera_rmat, scene_projector_to_camera_translation_vector, 1, camera_tvec, 1, scene_projector_to_camera_translation_vector, 0)
            print "Scene Translation Vector:"
            for row in range(scene_projector_to_camera_translation_vector.height):
                for col in range(scene_projector_to_camera_translation_vector.width):
                    print scene_projector_to_camera_translation_vector[row, col],
                print
            print            

            cv.Add(scene_projector_to_camera_translation_vector, self.projector_to_camera_translation_vector, self.projector_to_camera_translation_vector)
            cv.Add(scene_projector_to_camera_rotation_vector, self.projector_to_camera_rotation_vector, self.projector_to_camera_rotation_vector)

        cv.ConvertScale(self.projector_to_camera_translation_vector, self.projector_to_camera_translation_vector, scale=1.0/self.number_of_scenes)
        cv.ConvertScale(self.projector_to_camera_rotation_vector, self.projector_to_camera_rotation_vector, scale=1.0/self.number_of_scenes)
        
        print "Final Translation Vector:"
        for row in range(self.projector_to_camera_translation_vector.height):
            for col in range(self.projector_to_camera_translation_vector.width):
                print self.projector_to_camera_translation_vector[row, col],
            print
        print

        print "Final Rotation Vector:"
        for row in range(self.projector_to_camera_rotation_vector.height):
            for col in range(self.projector_to_camera_rotation_vector.width):
                print self.projector_to_camera_rotation_vector[row, col],
            print

    def capture(self):
        blank = self.get_picture_of_projection(self.blank_projection)
        positive = self.get_picture_of_projection(self.positive_chessboard_projection)
        negative = self.get_picture_of_projection(self.negative_chessboard_projection)

        difference = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
        cv.Sub(positive, negative, difference)
        #cv.CmpS(difference, 0, difference, cv.CV_CMP_GT)

        cv.NamedWindow("blank", flags=0)
        cv.ShowImage("blank", blank)
        cv.WaitKey(300)

        cv.NamedWindow("difference", flags=0)
        cv.ShowImage("difference", difference)
        cv.WaitKey(300)
        
        camera_image_points_printed, object_points_printed = detect_chessboard(blank, self.printed_chessboard_corners_x, self.printed_chessboard_corners_y, self.printed_chessboard_spacing, self.printed_chessboard_spacing)
        if camera_image_points_printed is None:
            return False

        cv.UndistortPoints(camera_image_points_printed, camera_image_points_printed, self.camera_model.intrinsicMatrix(), self.camera_model.distortionCoeffs())
        homography = cv.CreateMat(3, 3, cv.CV_32FC1)
        cv.FindHomography(camera_image_points_printed, object_points_printed, homography)   
        
        camera_image_points_projected, dummy = detect_chessboard(difference, self.projected_chessboard_corners_x, self.projected_chessboard_corners_y, None, None)
        if camera_image_points_projected is None:
            return False
        object_points_projected = cv.CreateMat(1, self.projected_chessboard_corners_x * self.projected_chessboard_corners_y, cv.CV_32FC2)
        cv.UndistortPoints(camera_image_points_projected, object_points_projected, self.camera_model.intrinsicMatrix(), self.camera_model.distortionCoeffs())
        cv.PerspectiveTransform(object_points_projected, object_points_projected, homography)

        object_points_3d = cv.CreateMat(1, self.projected_chessboard_corners_x * self.projected_chessboard_corners_y, cv.CV_32FC3)
        
        x = cv.CreateMat(1, self.projected_chessboard_corners_x * self.projected_chessboard_corners_y, cv.CV_32FC1)
        y = cv.CreateMat(1, self.projected_chessboard_corners_x * self.projected_chessboard_corners_y, cv.CV_32FC1)
        cv.Split(object_points_projected, x, y, None, None)
        z = cv.CreateMat(1, self.projected_chessboard_corners_x * self.projected_chessboard_corners_y, cv.CV_32FC1)
        cv.SetZero(z)

        cv.Merge(x, y, z, None, object_points_3d)

        self.object_points.append(object_points_3d)
        self.camera_image_points.append(camera_image_points_projected)
        self.projector_image_points.append(self.get_projector_image_points())
        self.number_of_scenes += 1

        return True

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

def mat_to_list(mat):
    result = []
    for row in range(mat.height):
        for col in range(mat.width):
            result.append(mat[row, col])
    return result

def main():
    rospy.init_node('projector_camera_calibrator')
    projector_camera_calibrator = ProjectorCameraCalibrator()
    while not rospy.is_shutdown():
        print str(projector_camera_calibrator.number_of_scenes) + " scenes captured so far."
        response = raw_input('Press C to capture or N to calibrate with the collected data: ')
        if response.upper() == 'C':
            if projector_camera_calibrator.capture() is False:
                print "Capture Error."
        elif response.upper() == 'N':
            projector_camera_calibrator.calibrate()
            response = raw_input('Press S to save this calibration, or N to continue capturing: ')
            if response.upper() == 'S':
                projector_camera_calibrator.write_projector_camera_info()
                break;
        else:
            print "Invalid response."
    

if __name__ == '__main__':
    main()
