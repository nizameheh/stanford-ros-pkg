#!/usr/bin/env python

import roslib
roslib.load_manifest('projector')
import rospy
import threading
import wx
import cv
import yaml
from cv_bridge import CvBridge, CvBridgeError
import projector.srv
import sensor_msgs.msg
import sensor_msgs.srv
        
class Projector(wx.Frame):
    def __init__(self):
        display_index = rospy.get_param('~display_index')
        self.projector_info_file = rospy.get_param('~projector_info_file')
        self.delay = rospy.get_param('~delay')
        display = wx.Display(display_index)
        display_geometry = display.GetGeometry()
        wx.Frame.__init__(self, None, title='Projection', pos=display_geometry.GetTopLeft(), size=display_geometry.GetSize())
        self.panel = wx.Panel(self)


        self.read_projector_info()
        self.projector_info.height = display_geometry.Height
        self.projector_info.width = display_geometry.Width

        self.bridge = CvBridge()
        self.projection = None
        self.mutex = threading.RLock()
        self.paint_flag = threading.Event()
        self.panel.Bind(wx.EVT_PAINT, self.on_paint)


        #This binding allows the projector to be quit if the user
        #accidentally sets the projector to use the main display
        self.panel.Bind(wx.EVT_LEFT_DOWN, self.on_mouse_down)        
        
        self.ShowFullScreen(True)
        self.Show(True)

        self.set_projector_info_service = rospy.Service('~set_projector_info', sensor_msgs.srv.SetCameraInfo, self. set_projector_info)
        self.get_projector_info_service = rospy.Service('~get_projector_info', projector.srv.GetProjectorInfo, self.get_projector_info)
        self.set_projection_service = rospy.Service('~set_projection', projector.srv.SetProjection, self.set_projection)

    def read_projector_info(self):
        self.projector_info = sensor_msgs.msg.CameraInfo()
        try:
            input_stream = file(self.projector_info_file, 'r')
        except IOError:
            rospy.loginfo('Specified projector info file at ' + self.projector_info_file + ' does not exist.')
            return
        info_dict = yaml.load(input_stream)
        try:
            self.projector_info.height = info_dict['height']
            self.projector_info.width = info_dict['width']
            self.projector_info.D = info_dict['D']
            self.projector_info.K = info_dict['K']
            self.projector_info.R = info_dict['R']
            self.projector_info.P = info_dict['P']
        except (TypeError, KeyError):
            rospy.loginfo('Projector info file at ' + self.projector_info_file + ' is in an unrecognized format.')
            return
        rospy.loginfo('Projector info successfully read from ' + self.projector_info_file)
        rospy.loginfo(self.projector_info)
        
    def write_projector_info(self):
        info_dict = {}
        info_dict['height'] = self.projector_info.height
        info_dict['width'] = self.projector_info.width
        info_dict['D'] = self.projector_info.D
        info_dict['K'] = self.projector_info.K 
        info_dict['R'] = self.projector_info.R
        info_dict['P'] = self.projector_info.P 
        output_stream = file(self.projector_info_file, 'w')
        yaml.dump(info_dict, output_stream)
        rospy.loginfo('Projector info successfully written to ' + self.projector_info_file)

    def set_projector_info(self, req):
        rospy.loginfo('Recieved request to set projector info.')
        self.projector_info = req.camera_info
        self.write_projector_info()
        response = sensor_msgs.srv.SetCameraInfoResponse()
        response.success = True
        return response

    def get_projector_info(self, req):
        response = projector.srv.GetProjectorInfoResponse()
        response.projector_info = self.projector_info
        return response

    def on_paint(self, evt):
        with self.mutex:
            if self.projection is not None:
                rgb8_projection = self.bridge.imgmsg_to_cv(self.projection, "rgb8")
                image = wx.ImageFromBuffer(rgb8_projection.width, rgb8_projection.height, rgb8_projection.tostring())
                bitmap = image.ConvertToBitmap()
                dc = wx.BufferedPaintDC(self.panel, bitmap)
            self.paint_flag.set()


    def set_projection(self, req):
        with self.mutex:
            self.projection = req.image
            self.paint_flag.clear()

        #wxPython doesn't like having multiple threads invoking
        #GUI methods, so we have our ROS thread add an event
        #to the wxPython thread's event queue by wrapping the call
        #in wx.CallAfter
        wx.CallAfter(self.panel.Refresh)

        self.paint_flag.wait()
        response = projector.srv.SetProjectionResponse()
        response.success = True
        rospy.sleep(self.delay)
        return response

    def on_mouse_down(self, event):
        self.Destroy()         

def main():
    rospy.init_node('projector')
    application = wx.PySimpleApp()
    projector = Projector()

    application.MainLoop()

if __name__ == '__main__':
    main()
