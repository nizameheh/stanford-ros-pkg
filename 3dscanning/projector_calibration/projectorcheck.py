#!/usr/bin/env python

import roslib
roslib.load_manifest('projector_calibration')
import rospy
import cv
import projector.srv
import sensor_msgs.msg
import sensor_msgs.srv
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
import math

class ProjectorChecker:
    def __init__(self):
        grid_spacing = rospy.get_param('~grid_spacing')

        self.bridge = CvBridge()

        rospy.loginfo("Waiting for projector info service...")
        rospy.wait_for_service('projector/get_projector_info')
        rospy.loginfo("Projector info service found.")
        projector_info_service = rospy.ServiceProxy('projector/get_projector_info', projector.srv.GetProjectorInfo)
        
        rospy.loginfo("Waiting for projection setting service...")
        rospy.wait_for_service('projector/set_projection')
        rospy.loginfo("Projection setting service found.")
        self.set_projection = rospy.ServiceProxy('projector/set_projection', projector.srv.SetProjection)

        projector_info = projector_info_service().projector_info
        projector_model = image_geometry.PinholeCameraModel()
        projector_model.fromCameraInfo(projector_info)

        # Generate grid projections
        self.original_projection = cv.CreateMat(projector_info.height, projector_info.width, cv.CV_8UC1)
        for row in range(0, projector_info.height, grid_spacing):
            cv.Line(self.original_projection, (0, row), (projector_info.width - 1, row), 255)
        for col in range(0, projector_info.width, grid_spacing):
            cv.Line(self.original_projection, (col, 0), (col, projector_info.height - 1), 255)


        predistortmap_x = cv.CreateMat(projector_info.height, projector_info.width, cv.CV_32FC1)
        predistortmap_y = cv.CreateMat(projector_info.height, projector_info.width, cv.CV_32FC1)
        InitPredistortMap(projector_model.intrinsicMatrix(), projector_model.distortionCoeffs(), predistortmap_x, predistortmap_y)


        self.predistorted_projection = cv.CreateMat(projector_info.height, projector_info.width, cv.CV_8UC1)
        cv.Remap(self.original_projection, self.predistorted_projection, predistortmap_x, predistortmap_y, flags=cv.CV_INTER_NN+cv.CV_WARP_FILL_OUTLIERS, fillval=(0, 0, 0, 0))

        self.off_projection = cv.CreateMat(projector_info.height, projector_info.width, cv.CV_8UC1)
        cv.SetZero(self.off_projection)

    def project(self, projection):
        projection_msg = self.bridge.cv_to_imgmsg(projection, encoding="mono8")
        self.set_projection(projection_msg)

def InitPredistortMap(cameraMatrix, distCoeffs, map1, map2):
    rows = map1.height
    cols = map1.width
    
    mat = cv.CreateMat(1, rows*cols, cv.CV_32FC2)

    for row in range(rows):
        for col in range(cols):
            mat[0, row * cols  + col] = (col, row)

    R = cv.CreateMat(3, 3, cv.CV_32FC1)
    cv.SetIdentity(R)

    P = cv.CreateMat(3, 4, cv.CV_64FC1)
    cv.SetZero(P)
    cv.Copy(cameraMatrix, cv.GetSubRect(P, (0, 0, 3, 3)))
    
    cv.UndistortPoints(mat, mat, cameraMatrix, distCoeffs, R, P)

    mat = cv.Reshape(mat, 2, rows)
        

    cv.Split(mat, map1, map2, None, None)

def main():
    rospy.init_node('projector_checker')
    checker = ProjectorChecker()
    while not rospy.is_shutdown():
        response = raw_input('Enter O to display original projection, P to display predistorted projection, or Q to quit: ')
        if response.upper() == 'O':
            checker.project(checker.original_projection)
        elif response.upper() == 'P':
            checker.project(checker.predistorted_projection)
        elif response.upper() == 'Q':
            # Turn projector off
            checker.project(checker.off_projection)
            break;
        else:
            print "Invalid response."
    

if __name__ == '__main__':
    main()
