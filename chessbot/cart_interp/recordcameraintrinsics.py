import roslib
roslib.load_manifest('cart_interp')
import rospy
from sensor_msgs.msg import CameraInfo
import time

def main():
    global cameras_recorded
    cameras_recorded = []
    rospy.init_node('camera_intrinsics_recorder')
    rate = rospy.Rate(5.0)
    cameras = ["wide_stereo/right", "wide_stereo/left", "narrow_stereo/right", "narrow_stereo/left"]
    for camera in cameras:
        camera = rospy.resolve_name(camera)
        rospy.Subscriber(camera + "/camera_info", CameraInfo, camera_callback_factory(camera, len(cameras)))

    last_count = -1
    while (len(cameras_recorded) < len(cameras) and not rospy.is_shutdown()):
        rate.sleep()
    print('complete')

def camera_callback_factory(camera, number_of_cameras):
    def camera_callback(camera_info):
        global cameras_recorded
        if (camera not in cameras_recorded):
            filename = '/home/davidmandle/test/'+camera.replace('/', '_')
            f = open(filename, 'w')
            f.write(str(camera_info))
            f.close()
            cameras_recorded.append(camera)
            print(str(len(cameras_recorded)) + ' of ' + str(number_of_cameras) + ' recorded')
    return camera_callback

if __name__ == '__main__':
    main()
