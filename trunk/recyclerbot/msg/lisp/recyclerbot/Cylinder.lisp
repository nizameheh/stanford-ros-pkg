; Auto-generated. Do not edit!


(in-package recyclerbot-msg)


;//! \htmlinclude Cylinder.msg.html

(defclass <Cylinder> (ros-message)
  ((pose
    :reader pose-val
    :initarg :pose
    :type geometry_msgs-msg:<Pose>
    :initform (make-instance 'geometry_msgs-msg:<Pose>))
   (height
    :reader height-val
    :initarg :height
    :type float
    :initform 0.0)
   (radius
    :reader radius-val
    :initarg :radius
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <Cylinder>) ostream)
  "Serializes a message object of type '<Cylinder>"
  (serialize (slot-value msg 'pose) ostream)
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'height))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'radius))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <Cylinder>) istream)
  "Deserializes a message object of type '<Cylinder>"
  (deserialize (slot-value msg 'pose) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'height) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'radius) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<Cylinder>)))
  "Returns string type for a message object of type '<Cylinder>"
  "recyclerbot/Cylinder")
(defmethod md5sum ((type (eql '<Cylinder>)))
  "Returns md5sum for a message object of type '<Cylinder>"
  "b9842a15b22cd9424e1012c80ae99e58")
(defmethod message-definition ((type (eql '<Cylinder>)))
  "Returns full string definition for message of type '<Cylinder>"
  (format nil "# A representation of cylinder in free space, composed of pose, height, and radius. ~%geometry_msgs/Pose pose~%float64 height~%float64 radius~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(defmethod serialization-length ((msg <Cylinder>))
  (+ 0
     (serialization-length (slot-value msg 'pose))
     8
     8
))
(defmethod ros-message-to-list ((msg <Cylinder>))
  "Converts a ROS message object to a list"
  (list '<Cylinder>
    (cons ':pose (pose-val msg))
    (cons ':height (height-val msg))
    (cons ':radius (radius-val msg))
))
