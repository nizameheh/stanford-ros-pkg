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
        self.projector_dimensions = (self.projector_info.width, self.projector_info.height)

        self.number_of_scanlines = []
        for i in range(2):
            self.number_of_scanlines.append(get_number_of_scanlines(self.projector_dimensions[i], self.pixels_per_scanline))

        self.read_scanner_info()

        rospy.loginfo("Generating projection patterns...")

        graycodes = []
        for i in range(max(self.number_of_scanlines)):
            graycodes.append(graycodemath.generate_gray_code(i))
            
        self.number_of_projection_patterns = []
        for i in range(2):
            self.number_of_projection_patterns.append(int(math.ceil(math.log(self.number_of_scanlines[i], 2))))
        
        self.positive_projections = []
        self.negative_projections = []
        for i in range(2):
            self.positive_projections.append([])
            self.negative_projections.append([])
            for j in range(self.number_of_projection_patterns[i]):
                cross_section = cv.CreateMat(1, self.projector_dimensions[i], cv.CV_8UC1)

                #Fill in cross section with the associated bit of each gray code
                for pixel in range(self.projector_dimensions[i]):
                    scanline = pixel // self.pixels_per_scanline
                    scanline_value = graycodemath.get_bit(graycodes[scanline], j) * 255
                    cross_section[0, pixel] = scanline_value

                #If we're doing horizontal scanning, transpose the cross section
                if i is 1:
                    cross_section_transpose = cv.CreateMat(self.projector_info.height, 1, cv.CV_8UC1)
                    cv.Transpose(cross_section, cross_section_transpose)
                    cross_section = cross_section_transpose

                #Repeat the cross section over the entire image
                positive_projection = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_8UC1)
                cv.Repeat(cross_section, positive_projection)

                #Create a negative of the pattern for thresholding
                negative_projection = cv.CreateMat(self.projector_info.height, self.projector_info.width, cv.CV_8UC1)
                cv.Not(positive_projection, negative_projection)
                
                #Fade the borders of the patterns to avoid artifacts at the edges of projection
                positive_projection_faded = fade_edges(positive_projection, 20)
                negative_projection_faded = fade_edges(negative_projection, 20)
                
                self.positive_projections[i].append(positive_projection_faded)
                self.negative_projections[i].append(negative_projection_faded)

        rospy.loginfo("Waiting for projection setting service...")
        rospy.wait_for_service('set_projection')
        rospy.loginfo("Projection setting service found.")
        self.set_projection = rospy.ServiceProxy('set_projection', projector.srv.SetProjection)
        self.point_cloud_msg = None

        rospy.Service("~get_point_cloud", graycode_scanner.srv.GetPointCloud, self.handle_point_cloud_srv)

        point_cloud_pub = rospy.Publisher('~point_cloud', sensor_msgs.msg.PointCloud)

        rospy.loginfo("Ready.")

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.pixel_associations_msg is not None:
                pixel_associations_pub.publish(self.pixel_associations_msg)
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
    
    #mode = 0 gets column associations
    #mode = 1 gets row associations
    def get_projector_line_associations(self, mode):
        rospy.loginfo("Scanning...")
        positives = []
        negatives = []
        for i in range(self.number_of_projection_patterns[mode]):
            positives.append(self.get_picture_of_projection(self.positive_projections[mode][i]))
            negatives.append(self.get_picture_of_projection(self.negative_projections[mode][i]))
        
            
        rospy.loginfo("Thresholding...")
        threshold_value = 8
        strike_sum = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_32SC1)
        cv.SetZero(strike_sum)
        gray_codes = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_32SC1)
        cv.SetZero(gray_codes)
        for i in range(self.number_of_projection_patterns[mode]):
            difference = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
            cv.Sub(positives[i], negatives[i], difference)

            absolute_difference = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
            cv.AbsDiff(positives[i], negatives[i], absolute_difference)

            #Mark all the pixels that were "too close to call" and add them to the running total
            strike_mask = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
            cv.CmpS(absolute_difference, threshold_value, strike_mask, cv.CV_CMP_LT)
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
        cv.InRangeS(projector_line_associations, 0, self.number_of_scanlines[mode], invalid_scanline_mask)
        cv.Not(invalid_scanline_mask, invalid_scanline_mask)
        cv.Set(projector_line_associations, -1, invalid_scanline_mask)
        
        self.display_scanline_associations(projector_line_associations, mode)

        return projector_line_associations

    def get_pixel_associations(self):
        col_associations = self.get_projector_line_associations(0)
        row_associations = self.get_projector_line_associations(1)
        
        rospy.loginfo("Merging data...")
    
        pixel_associations = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_32SC2)
        cv.Merge(col_associations, row_associations, None, None, pixel_associations)

        return pixel_associations

    def handle_point_cloud_srv(self, req):
        response = graycode_scanner.srv.GetPointCloudResponse()
        self.get_point_cloud()
        response.point_cloud = self.point_cloud_msg
        response.success = True
        return response

    def get_point_cloud(self):

        # Scan the scene
        pixel_associations = self.get_pixel_associations()

        # Project white light onto the scene to get the intensities of each picture (for coloring our point cloud)
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

        projector_model = PinholeCameraModel()
        projector_model.fromCameraInfo(self.projector_info)

        projector_to_camera_rotation_matrix = cv.CreateMat(3, 3, cv.CV_32FC1)
        cv.Rodrigues2(self.projector_to_camera_rotation_vector, projector_to_camera_rotation_matrix)
        
        pixel_associations_x = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_32SC1)
        pixel_associations_y = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_32SC1)
        cv.Split(pixel_associations, pixel_associations_x, pixel_associations_y, None, None)
        valid_points_mask_x = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
        cv.CmpS(pixel_associations_x, -1, valid_points_mask_x, cv.CV_CMP_NE)
        valid_points_mask_y = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
        cv.CmpS(pixel_associations_y, -1, valid_points_mask_y, cv.CV_CMP_NE)
        valid_points_mask = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
        cv.And(valid_points_mask_x, valid_points_mask_y, valid_points_mask)
        
        number_of_valid_points = cv.CountNonZero(valid_points_mask)

        rectified_camera_coordinates = cv.CreateMat(1, number_of_valid_points, cv.CV_32FC2)
        rectified_projector_coordinates = cv.CreateMat(1, number_of_valid_points, cv.CV_32FC2)
        intensities = cv.CreateMat(1, number_of_valid_points, cv.CV_8UC1)
        i = 0;
        for row in range(self.camera_info.height):
            for col in range(self.camera_info.width):
                if valid_points_mask[row, col] != 0:
                    rectified_camera_coordinates[0, i] = (col, row)
                    rectified_projector_coordinates[0, i] = pixel_associations[row, col]
                    intensities[0, i] = intensities_image[row, col]
                    i += 1

        cv.UndistortPoints(rectified_camera_coordinates, rectified_camera_coordinates, camera_model.intrinsicMatrix(), camera_model.distortionCoeffs())

        # Convert scanline numbers to pixel numbers
        cv.AddS(rectified_projector_coordinates, 0.5, rectified_projector_coordinates)
        cv.ConvertScale(rectified_projector_coordinates, rectified_projector_coordinates, self.pixels_per_scanline)

        # Rectify the projector pixels
        cv.UndistortPoints(rectified_projector_coordinates, rectified_projector_coordinates, projector_model.intrinsicMatrix(), projector_model.distortionCoeffs())

        camera_rays = projectPixelsTo3dRays(rectified_camera_coordinates, camera_model)

        projector_rays = projectPixelsTo3dRays(rectified_projector_coordinates, projector_model)

        # Bring the projector rays into camera coordinates
        cv.Transform(projector_rays, projector_rays,  projector_to_camera_rotation_matrix)

        camera_centers = cv.CreateMat(1, number_of_valid_points, cv.CV_32FC3)
        cv.SetZero(camera_centers)

        projector_centers = cv.CreateMat(1, number_of_valid_points, cv.CV_32FC3)
        cv.Set(projector_centers, self.projector_to_camera_translation_vector)
        
        intersection_points = line_line_intersections(camera_centers, camera_rays, projector_centers, projector_rays)
        
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
            points.append(point)
            intensity = intensities[0, i]
            intensities_list.append(intensity)
     
        channel.values = intensities_list
        channels.append(channel)
        self.point_cloud_msg.channels = channels
        self.point_cloud_msg.points = points

        return self.point_cloud_msg

    def scanline_number_to_pixel_number(self, scanline_number):
        return self.pixels_per_scanline * (scanline_number + 0.5)

    def display_scanline_associations(self, associations, mode):
        display_image = cv.CreateMat(self.camera_info.height, self.camera_info.width, cv.CV_8UC1)
        cv.ConvertScale(associations, display_image, 255.0/self.number_of_scanlines[mode])
        cv.NamedWindow("associations, " + str(mode), flags=0)
        cv.ShowImage("associations, " + str(mode), display_image)
        cv.WaitKey(800)

def print_mat(mat):
    for row in range(mat.height):
        for col in range(mat.width):
            print mat[row, col],
        print
    print

def element_wise_dot_product(a, b):
    a_as_row_vectors = cv.Reshape(a, 1, a.width * a.height)
    b_as_row_vectors = cv.Reshape(b, 1, b.width * b.height)
    a_as_rows_mul_b_as_rows = cv.CreateMat(a.width * a.height, 3, cv.CV_32FC1)
    cv.Mul(a_as_row_vectors, b_as_row_vectors, a_as_rows_mul_b_as_rows)
    dot_product = cv.CreateMat(a.width * a.height, 1, cv.CV_32FC1)
    cv.Reduce(a_as_rows_mul_b_as_rows, dot_product, dim=1, op=cv.CV_REDUCE_SUM)
    return cv.Reshape(dot_product, 1, a.height)

def projectPixelsTo3dRays(pixels, model):
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
    

def line_line_intersections(P_0, u, Q_0, v):
    rows = P_0.height
    cols = P_0.width
    w_0 = cv.CreateMat(rows, cols, cv.CV_32FC3)

    cv.Sub(P_0, Q_0, w_0)

    a = element_wise_dot_product(u, u)
    b = element_wise_dot_product(u, v)
    c = element_wise_dot_product(v, v)
    d = element_wise_dot_product(u, w_0)
    e = element_wise_dot_product(v, w_0)
    
    a_mul_c = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Mul(a, c, a_mul_c)

    b_squared = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Pow(b, b_squared, 2)
    
    denominator = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Sub(a_mul_c, b_squared, denominator)

    b_mul_e = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Mul(b, e, b_mul_e)

    c_mul_d = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Mul(c, d, c_mul_d)

    b_mul_d = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Mul(b, d, b_mul_d)

    a_mul_e = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Mul(a, e, a_mul_e)

    s_c = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Sub(b_mul_e, c_mul_d, s_c)
    cv.Div(s_c, denominator, s_c)
    
    t_c = cv.CreateMat(rows, cols, cv.CV_32FC1)
    cv.Sub(a_mul_e, b_mul_d, t_c)
    cv.Div(t_c, denominator, t_c)

    u_x = cv.CreateMat(rows, cols, cv.CV_32FC1)
    u_y = cv.CreateMat(rows, cols, cv.CV_32FC1)
    u_z = cv.CreateMat(rows, cols, cv.CV_32FC1)

    cv.Split(u, u_x, u_y, u_z, None)

    su_x = cv.CreateMat(rows, cols, cv.CV_32FC1)    
    su_y = cv.CreateMat(rows, cols, cv.CV_32FC1)    
    su_z = cv.CreateMat(rows, cols, cv.CV_32FC1)    

    cv.Mul(s_c, u_x, su_x)
    cv.Mul(s_c, u_y, su_y)
    cv.Mul(s_c, u_z, su_z)

    su = cv.CreateMat(rows, cols, cv.CV_32FC3)
    cv.Merge(su_x, su_y, su_z, None, su)

    tu_x = cv.CreateMat(rows, cols, cv.CV_32FC1)    
    tu_y = cv.CreateMat(rows, cols, cv.CV_32FC1)    
    tu_z = cv.CreateMat(rows, cols, cv.CV_32FC1)    

    cv.Mul(t_c, u_x, tu_x)
    cv.Mul(t_c, u_y, tu_y)
    cv.Mul(t_c, u_z, tu_z)

    tu = cv.CreateMat(rows, cols, cv.CV_32FC3)
    cv.Merge(tu_x, tu_y, tu_z, None, tu)

    closest_point = cv.CreateMat(rows, cols, cv.CV_32FC3)
    cv.Add(P_0, su, closest_point)
    return closest_point      

def list_to_matrix(list_input, rows, cols, matrix_type):
    mat = cv.CreateMat(rows, cols, matrix_type)
    for row in range(rows):
        for col in range(cols):
            mat[row, col] = list_input[(row * cols) + col]
    return mat
        
def get_number_of_scanlines(pixels, pixels_per_scanline):
        return int(math.ceil(float(pixels)/pixels_per_scanline))

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
