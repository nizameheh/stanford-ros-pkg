#!/usr/bin/env python

import math

import roslib
roslib.load_manifest('graycode_scanner')
import rospy
import cv
import threading
import projector.srv
import sensor_msgs.msg
import graycode_scanner.srv
import geometry_msgs.msg
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
import yaml

import graycodemath

class Scanner:
    def __init__(self):
        self.pixels_per_scanline = rospy.get_param('~pixels_per_scanline')
        self.scanner_info_file_name = rospy.get_param('~scanner_info_file_name')
        self.threshold = rospy.get_param('~threshold')        

        self.mutex = threading.RLock()
        self.image_update_flag = threading.Event()
        self.bridge = CvBridge()
        rospy.Subscriber("image_stream", sensor_msgs.msg.Image, self.update_image)
        rospy.loginfo("Waiting for camera info...")
        self.camera_info = rospy.wait_for_message('camera_info', sensor_msgs.msg.CameraInfo)
        rospy.loginfo("Camera info received.")

        rospy.loginfo("Waiting for projector info service...")
        rospy.wait_for_service('get_projector_info')
        rospy.loginfo("Projector info service found.")
        get_projector_info = rospy.ServiceProxy('get_projector_info', projector.srv.GetProjectorInfo)
        self.projector_info = get_projector_info().projector_info

        self.projector_model = PinholeCameraModel()
        self.projector_model.fromCameraInfo(self.projector_info)

        self.read_scanner_info()

        self.projector_to_camera_rotation_matrix = cv.CreateMat(3, 3, cv.CV_32FC1)
        cv.Rodrigues2(self.projector_to_camera_rotation_vector, self.projector_to_camera_rotation_matrix)        

        predistortmap_x = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_32FC1)
        predistortmap_y = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_32FC1)
        InitPredistortMap(self.projector_model.intrinsicMatrix(), self.projector_model.distortionCoeffs(), predistortmap_x, predistortmap_y)

        minVal, maxVal, minLoc, maxLoc = cv.MinMaxLoc(predistortmap_x)
        cv.AddS(predistortmap_x, -minVal, predistortmap_x)
        uncropped_projection_width = int(math.ceil(maxVal - minVal))
        self.center_pixel = self.projector_model.cx() - minVal

        minVal, maxVal, minLoc, maxLoc = cv.MinMaxLoc(predistortmap_y)
        cv.AddS(predistortmap_y, -minVal, predistortmap_y)
        uncropped_projection_height = int(math.ceil(maxVal - minVal))

        self.number_of_scanlines = int(math.ceil(float(uncropped_projection_width)/self.pixels_per_scanline))

        rospy.loginfo("Generating projection patterns...")

        graycodes = []
        for i in range(self.number_of_scanlines):
            graycodes.append(graycodemath.generate_gray_code(i))
    
        self.number_of_projection_patterns = int(math.ceil(math.log(self.number_of_scanlines, 2)))        
        self.predistorted_positive_projections = []
        self.predistorted_negative_projections = []

        for i in range(self.number_of_projection_patterns):
            cross_section = cv.CreateMat(1, uncropped_projection_width, cv.CV_8UC1)

            #Fill in cross section with the associated bit of each gray code
            for pixel in range(uncropped_projection_width):
                scanline = pixel // self.pixels_per_scanline
                scanline_value = graycodemath.get_bit(graycodes[scanline], i) * 255
                cross_section[0, pixel] = scanline_value

            #Repeat the cross section over the entire image
            positive_projection = cv.CreateMat(uncropped_projection_height, uncropped_projection_width, cv.CV_8UC1)
            cv.Repeat(cross_section, positive_projection)

            #Predistort the projections to compensate for projector optics so that that the scanlines are approximately planar
            predistorted_positive_projection = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_8UC1)
            cv.Remap(positive_projection, predistorted_positive_projection, predistortmap_x, predistortmap_y, flags=cv.CV_INTER_NN)

            #Create a negative of the pattern for thresholding
            predistorted_negative_projection = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_8UC1)
            cv.Not(predistorted_positive_projection, predistorted_negative_projection)
                
            #Fade the borders of the patterns to avoid artifacts at the edges of projection
            predistorted_positive_projection_faded = fade_edges(predistorted_positive_projection, 20)
            predistorted_negative_projection_faded = fade_edges(predistorted_negative_projection, 20)
                
            self.predistorted_positive_projections.append(predistorted_positive_projection_faded)
            self.predistorted_negative_projections.append(predistorted_negative_projection_faded)

        rospy.loginfo("Waiting for projection setting service...")
        rospy.wait_for_service('set_projection')
        rospy.loginfo("Projection setting service found.")
        self.set_projection = rospy.ServiceProxy('set_projection', projector.srv.SetProjection)

        self.pixel_associations_msg = None
        self.point_cloud_msg = None

        rospy.Service("~get_point_cloud", graycode_scanner.srv.GetPointCloud, self.handle_point_cloud_srv)

        point_cloud_pub = rospy.Publisher('~point_cloud', sensor_msgs.msg.PointCloud)

        rospy.loginfo("Ready.")

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.point_cloud_msg is not None:
                point_cloud_pub.publish(self.point_cloud_msg)
            rate.sleep()

    def read_scanner_info(self):
        try:
            input_stream = file(self.scanner_info_file_name, 'r')
        except IOError:
            rospy.loginfo('Specified scanner info file at ' + self.scanner_info_file_name + ' does not exist.')
            return
        info_dict = yaml.load(input_stream)
        try:
            self.projector_to_camera_translation_vector = tuple(info_dict['projector_to_camera_translation_vector'])
            self.projector_to_camera_rotation_vector = list_to_matrix(info_dict['projector_to_camera_rotation_vector'], 3, 1, cv.CV_32FC1)
        except (TypeError, KeyError):
            rospy.loginfo('Scanner info file at ' + self.scanner_info_file_name + ' is in an unrecognized format.')
            return
        rospy.loginfo('Scanner info successfully read from ' + self.scanner_info_file_name)

    def update_image(self, imagemsg):
        with self.mutex:
            if not self.image_update_flag.is_set():
                self.latest_image = self.bridge.imgmsg_to_cv(imagemsg, "mono8")
                self.image_update_flag.set()
    
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
    
    def get_projector_line_associations(self):
        rospy.loginfo("Scanning...")
        positives = []
        negatives = []
        for i in range(self.number_of_projection_patterns):
            positives.append(self.get_picture_of_projection(self.predistorted_positive_projections[i]))
            negatives.append(self.get_picture_of_projection(self.predistorted_negative_projections[i]))
        
            
        rospy.loginfo("Thresholding...")
        strike_sum = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_32SC1)
        cv.SetZero(strike_sum)
        gray_codes = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_32SC1)
        cv.SetZero(gray_codes)
        for i in range(self.number_of_projection_patterns):
            difference = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
            cv.Sub(positives[i], negatives[i], difference)

            absolute_difference = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
            cv.AbsDiff(positives[i], negatives[i], absolute_difference)

            #Mark all the pixels that were "too close to call" and add them to the running total
            strike_mask = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
            cv.CmpS(absolute_difference, self.threshold, strike_mask, cv.CV_CMP_LT)
            strikes = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_32SC1)
            cv.Set(strikes, 1, strike_mask)
            cv.Add(strikes, strike_sum, strike_sum)
            
            #Set the corresponding bit in the gray_code
            bit_mask = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
            cv.CmpS(difference, 0, bit_mask, cv.CV_CMP_GT)
            bit_values = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_32SC1)
            cv.Set(bit_values, 2 ** i, bit_mask)
            cv.Or(bit_values, gray_codes, gray_codes)

        rospy.loginfo("Decoding...")
        # Decode every gray code into binary
        projector_line_associations = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_32SC1)
        cv.Copy(gray_codes, projector_line_associations)
        for i in range(cv.CV_MAT_DEPTH(cv.GetElemType(projector_line_associations)), -1, -1):
            projector_line_associations_bitshifted_right = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_32SC1)
            #Using ConvertScale with a shift of -0.5 to do integer division for bitshifting right
            cv.ConvertScale(projector_line_associations, projector_line_associations_bitshifted_right, (2 ** -(2 ** i)), -0.5)
            cv.Xor(projector_line_associations, projector_line_associations_bitshifted_right, projector_line_associations)
        
        rospy.loginfo("Post processing...")
        
        # Remove all pixels that were "too close to call" more than once
        strikeout_mask = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
        cv.CmpS(strike_sum, 1, strikeout_mask, cv.CV_CMP_GT)
        cv.Set(projector_line_associations, -1, strikeout_mask)
        
        # Remove all pixels that don't decode to a valid scanline number
        invalid_scanline_mask = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
        cv.InRangeS(projector_line_associations, 0, self.number_of_scanlines, invalid_scanline_mask)
        cv.Not(invalid_scanline_mask, invalid_scanline_mask)
        cv.Set(projector_line_associations, -1, invalid_scanline_mask)
        
        self.display_scanline_associations(projector_line_associations)

        return projector_line_associations

    def handle_point_cloud_srv(self, req):
        response = graycode_scanner.srv.GetPointCloudResponse()
        self.get_point_cloud()
        response.point_cloud = self.point_cloud_msg
        response.success = True
        return response

    def get_point_cloud(self):

        # Scan the scene
        col_associations = self.get_projector_line_associations()

        # Project white light onto the scene to get the intensities of each pixel (for coloring our point cloud)
        illumination_projection = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_8UC1)
        cv.Set(illumination_projection, 255)
        intensities_image = self.get_picture_of_projection(illumination_projection)

        # Turn projector off when done with it
        off_projection = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_8UC1)
        cv.SetZero(off_projection)
        off_image_message = self.bridge.cv_to_imgmsg(off_projection, encoding="mono8")
        self.set_projection(off_image_message)

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(self.camera_info)
        
        valid_points_mask = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
        cv.CmpS(col_associations, -1, valid_points_mask, cv.CV_CMP_NE)
        
        number_of_valid_points = cv.CountNonZero(valid_points_mask)

        rectified_camera_coordinates = cv.CreateMat(1, number_of_valid_points, cv.CV_32FC2)
        scanline_associations = cv.CreateMat(1, number_of_valid_points, cv.CV_32FC1)
        intensities = cv.CreateMat(1, number_of_valid_points, cv.CV_8UC1)
        i = 0;
        for row in range(self.camera_info.height):
            for col in range(self.camera_info.width):
                if valid_points_mask[row, col] != 0:
                    rectified_camera_coordinates[0, i] = (col, row)
                    scanline_associations[0, i] = col_associations[row, col]
                    intensities[0, i] = intensities_image[row, col]
                    i += 1
       
        cv.UndistortPoints(rectified_camera_coordinates, rectified_camera_coordinates, camera_model.intrinsicMatrix(), camera_model.distortionCoeffs())

        # Convert scanline numbers to plane normal vectors
        planes = self.scanline_numbers_to_planes(scanline_associations)

        camera_rays = project_pixels_to_3d_rays(rectified_camera_coordinates, camera_model)
        
        intersection_points = ray_plane_intersections(camera_rays, planes)
        
        self.point_cloud_msg = sensor_msgs.msg.PointCloud()
        self.point_cloud_msg.header.stamp = rospy.Time.now()
        self.point_cloud_msg.header.frame_id = 'base_link'
        channels = []
        channel = sensor_msgs.msg.ChannelFloat32()
        channel.name = "intensity"
        points = []
        intensities_list = []

        for i in range(number_of_valid_points):
            point = geometry_msgs.msg.Point32()
            point.x = intersection_points[0, i][0]
            point.y = intersection_points[0, i][1]
            point.z = intersection_points[0, i][2]
            if point.z < 0:
                print scanline_associations[0, i]
                print rectified_camera_coordinates[0, i]
                
            points.append(point)
            intensity = intensities[0, i]
            intensities_list.append(intensity)
     
        channel.values = intensities_list
        channels.append(channel)
        self.point_cloud_msg.channels = channels
        self.point_cloud_msg.points = points

        return self.point_cloud_msg

    def scanline_numbers_to_planes(self, scanline_numbers):
        rows = scanline_numbers.height
        cols = scanline_numbers.width
        normal_vectors_x = cv.CreateMat(rows, cols, cv.CV_32FC1)
        cv.Set(normal_vectors_x, -1)
        normal_vectors_y = cv.CreateMat(rows, cols, cv.CV_32FC1)
        cv.Set(normal_vectors_y, 0)
        normal_vectors_z = cv.CreateMat(rows, cols, cv.CV_32FC1)
        cv.Copy(scanline_numbers, normal_vectors_z)

        cv.ConvertScale(normal_vectors_z, normal_vectors_z, scale=self.pixels_per_scanline)
        cv.AddS(normal_vectors_z, -self.center_pixel, normal_vectors_z)
        cv.ConvertScale(normal_vectors_z, normal_vectors_z, scale=1.0/self.projector_model.fx())
        
        normal_vectors = cv.CreateMat(rows, cols, cv.CV_32FC3)
        cv.Merge(normal_vectors_x, normal_vectors_y, normal_vectors_z, None, normal_vectors)

        # Bring the normal vectors into camera coordinates
        cv.Transform(normal_vectors, normal_vectors, self.projector_to_camera_rotation_matrix)
        
        normal_vectors_split = [None]*3
        for i in range(3):
            normal_vectors_split[i] = cv.CreateMat(rows, cols, cv.CV_32FC1)
        cv.Split(normal_vectors, normal_vectors_split[0], normal_vectors_split[1], normal_vectors_split[2], None)

        n_dot_p = cv.CreateMat(rows, cols, cv.CV_32FC1)
        cv.SetZero(n_dot_p)
        for i in range(3):
            cv.ScaleAdd(normal_vectors_split[i], self.projector_to_camera_translation_vector[i], n_dot_p, n_dot_p)

        planes = cv.CreateMat(rows, cols, cv.CV_32FC4)
        cv.Merge(normal_vectors_split[0], normal_vectors_split[1], normal_vectors_split[2], n_dot_p, planes)
        

        return planes

    def display_scanline_associations(self, associations):
        display_image = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
        cv.ConvertScale(associations, display_image, 255.0/self.number_of_scanlines)
        cv.NamedWindow("associations", flags=0)
        cv.ShowImage("associations", display_image)
        cv.WaitKey(800)

#This function assumes the vectors pass through the origin
def ray_plane_intersections(rays, planes):
    rows = rays.height
    cols = rays.width

    rays_split = [None]*3
    for i in range(3):
        rays_split[i] = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Split(rays, rays_split[0], rays_split[1], rays_split[2], None)

    planes_split = [None]*4
    for i in range(4):
        planes_split[i] = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Split(planes, planes_split[0], planes_split[1], planes_split[2], planes_split[3])

    n_dot_v = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.SetZero(n_dot_v)
    for i in range(3):
        temp = cv.CreateMat(rows, cols, cv.CV_32FC1)
        cv.Mul(planes_split[i], rays_split[i], temp)
        cv.Add(temp, n_dot_v, n_dot_v)
    depth = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Div(planes_split[3], n_dot_v, depth)
    
    intersection_points_split = [None]*3
    for i in range(3):
        intersection_points_split[i] = cv.CreateMat(rows, cols, cv.CV_32FC1)
    
    for i in range(3):
        cv.Mul(depth, rays_split[i], intersection_points_split[i])

    intersection_points = cv.CreateMat(rows, cols, cv.CV_32FC3)
    cv.Merge(intersection_points_split[0], intersection_points_split[1], intersection_points_split[2], None, intersection_points)
   
    return intersection_points

def print_mat(mat):
    for row in range(mat.height):
        for col in range(mat.width):
            print mat[row, col],
        print
    print

def project_pixels_to_3d_rays(pixels, model):
    x = cv.CreateMat(pixels.height, pixels.width, cv.CV_32FC1)
    y = cv.CreateMat(pixels.height, pixels.width, cv.CV_32FC1)
    cv.Split(pixels, x, y, None, None)

    x_squared = cv.CreateMat(pixels.height, pixels.width, cv.CV_32FC1)
    cv.Pow(x, x_squared, 2)

    y_squared = cv.CreateMat(pixels.height, pixels.width, cv.CV_32FC1)
    cv.Pow(y, y_squared, 2)

    inverse_norm = cv.CreateMat(pixels.height, pixels.width, cv.CV_32FC1)
    cv.Add(x_squared, y_squared, inverse_norm)
    cv.AddS(inverse_norm, 1, inverse_norm)
    cv.Pow(inverse_norm, inverse_norm, -0.5)

    cv.Mul(x, inverse_norm, x)
    cv.Mul(y, inverse_norm, y)

    result = cv.CreateMat(pixels.height, pixels.width, cv.CV_32FC3)
    cv.Merge(x, y, inverse_norm, None, result)
    return result

    
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

def list_to_matrix(list_input, rows, cols, matrix_type):
    mat = cv.CreateMat(rows, cols, matrix_type)
    for row in range(rows):
        for col in range(cols):
            mat[row, col] = list_input[(row * cols) + col]
    return mat
        
def fade_edges(projection_pattern, fade_width):
    divisor = cv.CreateMat(projection_pattern.height, projection_pattern.width, cv.CV_8UC1)
    cv.Set(divisor, 1)
    for i in range(fade_width):
        cv.Rectangle(divisor, (i, i), (projection_pattern.width - 1 - i, projection_pattern.height - 1 - i), int(((-5.0/fade_width) * i) + 6))
    cv.Div(projection_pattern, divisor, projection_pattern)
    return projection_pattern
    
def main():
    rospy.init_node('scanner')
    scanner = Scanner()

if __name__ == '__main__':
    main()
