""" Support for the PNI SpacePoint Fusion

Copyright (c) 2010, Nirav Patel <http://eclecti.cc>

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

This module provides support for the SpacePoint Fusion in quaternion mode 
using libhid.  It thus requires the python-hid wrapper.  Note that upon 
plugging in the SpacePoint, it should be kept still for a few seconds while 
the gyros calibrate.

SpacePoint -- The class which interfaces the SpacePoint Fusion
"""

__version__ = '0.2'
__author__ = 'Nirav Patel'

import sys
import hid

class SpacePoint:

    """Interface the SpacePoint Fusion in Quaternion mode
    
    start() -- Starts by default on init, but can be paired with stop()
    stop() -- Allows stopping the device, restart using start()
    update() -- Read in new data. The device refreshes at 62.5 Hz
    quat -- a tuple containing the quaternion
    accel -- a tuple containing x, y, and z accelerometer data
    buttons -- a tuple containing values for the two buttons on the device
    """
    
    def __init__(self):
        self._hid = 0
        self._running = False
        self.quat = None
        self.accel = None
        self.buttons = None
        
        self.start()
        self.update()
        
    def __repr__(self):
        return 'accel: %s quat: %s buttons: %s' % (repr(self.accel),
               repr(self.quat),repr(self.buttons))
        
    def __del__(self):
        if self._running:
            self.stop()
        
    def start(self):
        """Starts by default on init, but can be paired with stop()"""
        if not self._running:
            ret = hid.hid_init()
            if ret != hid.HID_RET_SUCCESS:
                sys.stderr.write("hid_init failed with return code %d.\n" % ret)
                return
        
            self._hid = hid.hid_new_HIDInterface()
            matcher = hid.HIDInterfaceMatcher()
            matcher.vendor_id = 0x20ff
            matcher.product_id = 0x0100

            # The following is necessary due to a bug in the firmware. Units
            # shipped after early February 2010 will likely have the fix and
            # should successfully open on interface 1.  Older units have to 
            # open on interface 1, fail, and then open on interface 0.
            ret = hid.hid_force_open(self._hid, 1, matcher, 3)
            if ret != hid.HID_RET_SUCCESS:
                ret = hid.hid_force_open(self._hid, 0, matcher, 3)
            if ret != hid.HID_RET_SUCCESS:
                sys.stderr.write("hid_force_open failed with return code %d.\n" % ret)
                return
            
            self._running = True
          
    def stop(self):
        """Allows stopping the device, restart using start()"""
        if self._running:
            ret = hid.hid_close(self._hid)
            if ret != hid.HID_RET_SUCCESS:
                sys.stderr.write("hid_close failed with return code %d.\n" % ret)
              
            self._running = False
            
            hid.hid_cleanup()
        
    def update(self):
        """Read in new data. The device refreshes at 62.5 Hz"""
        if not self._running:
          return
    
        ret,data = hid.hid_interrupt_read(self._hid,0x82,15,0);
        if ret != hid.HID_RET_SUCCESS:
            sys.stderr.write("hid_interrupt_read failed with return code %d.\n" % ret)
        else:
            # see: http://www.pnicorp.com/support/notes/spacepoint-fusion
            x = ((ord(data[0])|(ord(data[1])<<8))-32768)*0.000183105469
            y = ((ord(data[2])|(ord(data[3])<<8))-32768)*0.000183105469
            z = ((ord(data[4])|(ord(data[5])<<8))-32768)*0.000183105469
            self.accel = (x,y,z)
            q0 = ((ord(data[6])|(ord(data[7])<<8))-32768)/32768.0
            q1 = ((ord(data[8])|(ord(data[9])<<8))-32768)/32768.0
            q2 = ((ord(data[10])|(ord(data[11])<<8))-32768)/32768.0
            q3 = ((ord(data[12])|(ord(data[13])<<8))-32768)/32768.0
            self.quat = (q0,q1,q2,q3)
            b0 = ord(data[14])&1;
            b1 = (ord(data[14])&2)>>1;
            self.buttons = (b0,b1)
            
if __name__ == '__main__':
    fusion = SpacePoint()
    while 1:
      print repr(fusion)
      fusion.update()
