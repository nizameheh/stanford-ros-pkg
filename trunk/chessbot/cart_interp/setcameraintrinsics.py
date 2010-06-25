import roslib
import rospy
import actionlib
import sensor_msgs.msg
import sensor_msgs.srv

def main():
    cam_info = sensor_msgs.msg.CameraInfo()
    cam_info.header.stamp = rospy.Time.now()
    cam_info.header.frame_id = 

    cam_info.height = 
    cam_info.width = 

    cam_info.roi.x_offset = 
    cam_info.roi.y_offset =
    cam_info.roi.height = 
    cam_info.roi.width =

    cam_info.D =
    cam_info.K =
    cam_info.R =
    cam_info.P =

    cam_name = "wide_stereo/right"
    #cam_name = "wide_stereo/left"
    #cam_name = "narrow_stereo/right"
    #cam_name = "narrow_stereo/left"

    cam_name = rospy.resolve_name(cam_name)
    set_cal_service_name = cam_name + "/set_camera_info"

    print "Waiting for service [%s] to be available" % set_cal_service_name 
    rospy.wait_for_service(set_cal_service_name)
    print "Writing camera info to camera memory..."
    set_cal_srv = rospy.ServiceProxy(set_cal_service_name, sensor_msgs.srv.SetCameraInfo)
    resp = set_cal_srv(cam_info)
    if resp.success:
        print "Done writing to camera"
    else:
        print "**************** ERROR WRITING TO CAMERA. PLEASE RETRY ***********************"
        sys.exit(-1)


if __name__ == '__main__':
    main()
