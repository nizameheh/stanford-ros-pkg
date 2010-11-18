; Auto-generated. Do not edit!


(in-package recyclerbot-msg)


;//! \htmlinclude CylinderArray.msg.html

(defclass <CylinderArray> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (cylinders
    :reader cylinders-val
    :initarg :cylinders
    :type (vector recyclerbot-msg:<Cylinder>)
   :initform (make-array 0 :element-type 'recyclerbot-msg:<Cylinder> :initial-element (make-instance 'recyclerbot-msg:<Cylinder>))))
)
(defmethod serialize ((msg <CylinderArray>) ostream)
  "Serializes a message object of type '<CylinderArray>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'cylinders))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (serialize ele ostream))
    (slot-value msg 'cylinders))
)
(defmethod deserialize ((msg <CylinderArray>) istream)
  "Deserializes a message object of type '<CylinderArray>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'cylinders) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'cylinders)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (make-instance 'recyclerbot-msg:<Cylinder>))
(deserialize (aref vals i) istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<CylinderArray>)))
  "Returns string type for a message object of type '<CylinderArray>"
  "recyclerbot/CylinderArray")
(defmethod md5sum ((type (eql '<CylinderArray>)))
  "Returns md5sum for a message object of type '<CylinderArray>"
  "146785251b817b1cf46852ac40c74ac7")
(defmethod message-definition ((type (eql '<CylinderArray>)))
  "Returns full string definition for message of type '<CylinderArray>"
  (format nil "# An array of poses with a header for global reference.~%~%Header header~%~%recyclerbot/Cylinder[] cylinders~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: recyclerbot/Cylinder~%# A representation of cylinder in free space, composed of pose, height, and radius. ~%geometry_msgs/Pose pose~%float64 height~%float64 radius~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(defmethod serialization-length ((msg <CylinderArray>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (reduce #'+ (slot-value msg 'cylinders) :key #'(lambda (ele) (declare (ignorable ele)) (+ (serialization-length ele))))
))
(defmethod ros-message-to-list ((msg <CylinderArray>))
  "Converts a ROS message object to a list"
  (list '<CylinderArray>
    (cons ':header (header-val msg))
    (cons ':cylinders (cylinders-val msg))
))
